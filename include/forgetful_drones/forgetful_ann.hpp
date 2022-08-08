#pragma once
#include <string>
#include <torch/script.h>
#include "forgetful_drones/forgetful_helpers.hpp"


namespace forgetful_drone {

class ForgetfulANN {

    public: 

        ForgetfulANN () 
            :
            m_RGBHeight {},
            m_RGBWidth {},
            m_OPTMask {},
            m_OPTSize {},
            m_GRUNumLayers {},
            m_GRUHiddenSize {},
            m_TorchDevice {},
            m_TorchTensorOptions {}
        {};


    private:

        static constexpr uint BATCH_SIZE = 1;
        static constexpr uint SEQUENCE_LENGTH = 1;
        static constexpr std::array<std::tuple<const char*, c10::DeviceType>, 2> h_BRAIN_TORCHDEVICES {
            std::make_tuple("CPU", torch::kCPU),
            std::make_tuple("CUDA", torch::kCUDA),
        };

        const uint m_RGBHeight;
        const uint m_RGBWidth;
        const std::vector<bool> m_OPTMask;
        const uint m_OPTSize;
        float* m_optPtr;
        const uint m_GRUNumLayers;
        const uint m_GRUHiddenSize;
        const c10::DeviceType m_TorchDevice;
        const torch::TensorOptions m_TorchTensorOptions;
    
        torch::jit::script::Module m_TorchScriptModule;
        std::vector<float> m_GRUHiddenState;

    public:

        bool init (
            const uint& rgb_height,
            const uint& rgb_width,
            const std::vector<bool>& opt_mask,
            const uint& gru_num_layers, 
            const uint& gru_hidden_size,
            const uint& torch_device,
            const std::string& script_module_fpath
        ) {
            const_cast<uint&>(m_RGBHeight) = rgb_height;
            const_cast<uint&>(m_RGBWidth) = rgb_width;

            const_cast<std::vector<bool>&>(m_OPTMask) = opt_mask;
            const_cast<uint&>(m_OPTSize) = 0;
            for (const bool& x : m_OPTMask) if (x) const_cast<uint&>(m_OPTSize) +=1;

            const_cast<uint&>(m_GRUNumLayers) = gru_num_layers;
            const_cast<uint&>(m_GRUHiddenSize) = gru_hidden_size;
            initHiddenState ();

            const_cast<c10::DeviceType&>(m_TorchDevice) 
                = std::get<1>(h_BRAIN_TORCHDEVICES[torch_device]);
            const_cast<torch::TensorOptions&>(m_TorchTensorOptions)
                = torch::TensorOptions()
                    .dtype(torch::kFloat32)
                    .layout(torch::kStrided)
                    .requires_grad(false);

            try {
                m_TorchScriptModule = torch::jit::load (script_module_fpath);
                m_TorchScriptModule.to (torch::Device (m_TorchDevice, 0));
            } catch (const c10::Error& e) {
                ROSERROR("Error loading model: " << e.what());
                return false;
            } catch(const std::exception& e) {
                ROSERROR(e.what());
                return false;
            }

            std::cout << std::endl;
            ROSINFO("Forgetful ANN" << std::endl
                << "\t\t- SCRIPT MODULE:   " << script_module_fpath << std::endl
                << "\t\t- TORCH DEVICE:    " << std::get<0>(h_BRAIN_TORCHDEVICES[torch_device]) << std::endl
                << "\t\t- RGB SIZE:        " <<  m_RGBHeight << " x " << m_RGBWidth << std::endl
                << "\t\t- OPT MASK:        " <<  m_OPTMask << ",      size: " <<  m_OPTSize << std::endl
                << "\t\t- GRU # LAYERS:    " << m_GRUNumLayers << std::endl
                << "\t\t- GRU HIDDEN SIZE: " << m_GRUHiddenSize
            );
            std::cout << std::endl;

            return true;
        }

        void initHiddenState () {
            m_GRUHiddenState = std::vector <float> (
                m_GRUNumLayers * BATCH_SIZE * m_GRUHiddenSize, 0.0);
        };

        template <size_t N>
        Eigen::Vector3d forward (const cv::Mat& rgb_raw, const float (&inp_opt)[N]) {

            at::Tensor x_rgb = tensorFromRGB (preprocessRGB (rgb_raw));
            preprocessOPT (inp_opt); at::Tensor x_opt = tensorFromOpt ();
            at::Tensor h = tensorFromHidden ();

            std::vector <torch::jit::IValue> input {x_rgb, x_opt, h};
            auto output = m_TorchScriptModule.forward (input).toTuple ();
            free (m_optPtr);
            


            torch::Tensor output_x = output->elements () [0].toTensor ().cpu ().view ({3});
            torch::Tensor output_h = output->elements () [1].toTensor ().cpu ().view (
                {m_GRUNumLayers * BATCH_SIZE * m_GRUHiddenSize}
            );

            auto acc_h = output_h.accessor <float, 1> ();
            for (int i = 0; i < acc_h.size (0); i++) m_GRUHiddenState [i] = acc_h [i];
                
            //auto acc_x = output_x.accessor <float, 2> ();
            //Eigen::Vector3d out {acc_x [0] [0], acc_x [0] [1], acc_x [0] [2]};

            auto acc_x = output_x.accessor <float, 1> ();
            return {(double) acc_x [0], (double) acc_x [1], (double) acc_x [2]};
        };

    
    private:

        cv::Mat preprocessRGB (const cv::Mat& raw) {

            //ROSINFO (GET_VAR_REP (raw.rows) << " x " << GET_VAR_REP (raw.cols));
    
            cv::Mat resized; cv::resize (
                raw, 
                resized, 
                cv::Size (m_RGBWidth, m_RGBHeight)
            );

            //ROSINFO (GET_VAR_REP (resized.rows) << " x " << GET_VAR_REP (resized.cols));

            cv::Mat converted; cv::cvtColor (
                resized, 
                converted, 
                cv::COLOR_BGR2RGB
            );


            cv::Mat normalized; converted.convertTo (
                normalized, CV_32F, 1.0 / 255
            );

            return normalized;
        }

        template <size_t N>
        void preprocessOPT (const float (&in)[N]) {
            m_optPtr = (float *) malloc (m_OPTSize * sizeof (*m_optPtr));

            size_t j = 0;
            for (size_t i = 0; i < N; i++) 
                if (m_OPTMask [i])
                    m_optPtr [j++] = in [i];
        };



        at::Tensor tensorFromRGB (const cv::Mat& in) {
            at::Tensor out = torch::from_blob (
                in.data,
                {SEQUENCE_LENGTH, BATCH_SIZE, in.rows, in.cols, in.channels ()},
                m_TorchTensorOptions
            );
            out = out.permute ({0, 1, 4, 2, 3});
            
            return out.to (torch::Device (m_TorchDevice, 0));
        }

        at::Tensor tensorFromOpt () {
            at::Tensor out = torch::from_blob (
                m_optPtr,
                {SEQUENCE_LENGTH, BATCH_SIZE, m_OPTSize},
                m_TorchTensorOptions
            );
            
            return out.to (torch::Device (m_TorchDevice, 0));
        }

        at::Tensor tensorFromHidden () {
            at::Tensor out = torch::from_blob (
                m_GRUHiddenState.data (),
                {m_GRUNumLayers, BATCH_SIZE, m_GRUHiddenSize},
                m_TorchTensorOptions
            );
            
            return out.to (torch::Device (m_TorchDevice, 0));
        }


};


}