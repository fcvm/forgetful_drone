#pragma once
#include <string>
#include <torch/script.h>
#include "forgetful_drones/forgetful_helpers.hpp"




class ForgetfulBrain {
    public:
        ForgetfulBrain (
            const std::string& torch_scriptmodulefpath,
            const bool& torch_cuda,
            const uint& rgb_height,
            const uint& rgb_width,
            const uint& opt_size,
            const uint& gru_hiddensize, 
            const uint& gru_numlayers
        )
            :
            m_TorchDevice {torch_cuda? torch::kCUDA : torch::kCPU},
            m_TorchTensorOptions {torch::TensorOptions()
                .dtype(torch::kFloat32)
                .layout(torch::kStrided)
                .requires_grad(false)
            },
            m_RGBHeight {rgb_height},
            m_RGBWidth {rgb_width},
            m_OPTSize {opt_size},
            m_GRUNumLayers {gru_numlayers},
            m_GRUHiddenSize {gru_hiddensize}
        {
            try {
                m_TorchScriptModule = torch::jit::load(torch_scriptmodulefpath);
                m_TorchScriptModule.to(torch::Device(m_TorchDevice, 0));
            } catch (const c10::Error& e) {
                ROSERROR("Error loading model: " << e.what());
            } catch(const std::exception& e) {
                ROSERROR(e.what());
            }

            m_GRUHiddenState = std::vector<float>(
                m_GRUNumLayers * BATCH_SIZE * m_GRUHiddenSize, 0.0);
        }


        Eigen::Vector3d inferNavigationDecision (
            const cv::Mat& raw_rgb,
            const std::vector<float>& opt_input
        ) {
            at::Tensor x_rgb = tensorFromRGB (preprocessRGB (raw_rgb));
            at::Tensor x_opt = tensorFromOpt (opt_input);
            at::Tensor h = tensorFromHidden ();

            std::vector<torch::jit::IValue> input {x_rgb, x_opt, h};
            auto output = m_TorchScriptModule.forward(input).toTuple();

            torch::Tensor output_x = output->elements()[0].toTensor().cpu();
            torch::Tensor output_h = output->elements()[1].toTensor().cpu().view(
                {m_GRUNumLayers * BATCH_SIZE * m_GRUHiddenSize}
            );

            auto acc_h = output_h.accessor<float, 1>();
            for (int i = 0; i < acc_h.size(0); i++) m_GRUHiddenState[i] = acc_h[i];
                
            auto acc_x = output_x.accessor<float, 2>();
            return {acc_x[0][0], acc_x[0][1], acc_x[0][2]};
        };


    private:
        static constexpr uint BATCH_SIZE = 1;
        static constexpr uint SEQUENCE_LENGTH = 1;

        const uint m_RGBHeight;
        const uint m_RGBWidth;
        const uint m_OPTSize;
        const uint m_GRUNumLayers;
        const uint m_GRUHiddenSize;
        const c10::DeviceType m_TorchDevice;
        const torch::TensorOptions m_TorchTensorOptions;
        
        torch::jit::script::Module m_TorchScriptModule;
        std::vector<float> m_GRUHiddenState;


        cv::Mat preprocessRGB (const cv::Mat& raw) {
            cv::Mat resized; cv::resize(
                raw, 
                resized, 
                cv::Size(m_RGBWidth, m_RGBHeight)
            );

            cv::Mat converted; cv::cvtColor(
                resized, 
                converted, 
                cv::COLOR_BGR2RGB
            );

            cv::Mat normalized; converted.convertTo(
                normalized, CV_32F, 1.0 / 255
            );

            return normalized;
        }

        at::Tensor tensorFromRGB (const cv::Mat& in) {
            at::Tensor out = torch::from_blob(
                in.data,
                {SEQUENCE_LENGTH, BATCH_SIZE, in.rows, in.cols, in.channels()},
                m_TorchTensorOptions
            );
            out = out.permute({0, 1, 4, 2, 3});
            
            return out.to(torch::Device(m_TorchDevice, 0));
        }

        at::Tensor tensorFromOpt (const std::vector<float>& in) {
            at::Tensor out = torch::from_blob(
                const_cast<std::vector<float>&>(in).data(),
                {SEQUENCE_LENGTH, BATCH_SIZE, (signed long) in.size()},
                m_TorchTensorOptions
            );
            
            return out.to(torch::Device(m_TorchDevice, 0));
        }

        at::Tensor tensorFromHidden () {
            at::Tensor out = torch::from_blob(
                m_GRUHiddenState.data(), 
                {m_GRUNumLayers, BATCH_SIZE, m_GRUHiddenSize},
                m_TorchTensorOptions
            );
            
            return out.to(torch::Device(m_TorchDevice, 0));
        }




};