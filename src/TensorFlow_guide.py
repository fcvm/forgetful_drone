#!/usr/bin/env python


def data_input_pipelines_guide():

    #############################################
    # tf.data: Build TensorFlow input pipelines #
    #############################################
    import tensorflow as tf
    import pathlib
    import os
    import matplotlib.pyplot as plt
    import pandas as pd
    import numpy as np
    np.set_printoptions(precision=4)

    def basic_mechanics():
        # pipeline: data source -> tf.data.Dataset
        #   data source
        #     data in memory:
        #        tf.data.Dataset.from_tensors()
        #        tf.data.Dataset.from_tensor_slices()
        #     data in file (TFRecord format):
        #        tf.data.TFRecordDataset()
        #   tf.data.Dataset
        #     transformable into a new Dataset by chaining method calls on the tf.data.Dataset object
        #       per-element transformations such as Dataset.map()
        #       multi-element transformations such as Dataset.batch()

        # Dataset object is a Python iterable
        dataset = tf.data.Dataset.from_tensor_slices( [8, 3, 0, 8, 2, 1] )
        print( dataset )

        for elem in dataset:
            print( elem.numpy() )

        it = iter( dataset ) # Creating python iterator explicitly
        print( next(it).numpy() )

        # reduce transformation:
        #   reduces all elements to produce a single result
        #     example: compute the sum of a dataset of integers
        print( dataset.reduce(0, lambda state, value: state + value).numpy() )

        ###
        # **Dataset structure
        # dataset = sequence of elements with same structure
        #   (nested) structure of components
        #     Individual components can be of type tf.TypeSpec:
        #       tf.Tensor, 
        #       tf.sparse.SparseTensor, 
        #       tf.RaggedTensor, 
        #       tf.TensorArray
        #       tf.data.Dataset.
        #     Python constructs that can be used to express the (nested) structure:
        #       tuple, 
        #       dict, 
        #       NamedTuple,
        #       OrderedDict
        #     list is not a valid construct for expressing the structure
        #       convert into tuple to treat list input as a structure
        #       explicit packing with tf.stack to treat list output as single component

        # Dataset.element_spec: inspect the type of each element component
        #   returns a nested structure of tf.TypeSpec objects
        dataset1 = tf.data.Dataset.from_tensor_slices( tf.random.uniform([4, 10]) )
        print( dataset1.element_spec )

        dataset2 = tf.data.Dataset.from_tensor_slices( 
            (
                tf.random.uniform( [4] ),
                tf.random.uniform( [4, 100], maxval=100, dtype=tf.int32 )
            )
        )
        print( dataset2.element_spec )

        dataset3 = tf.data.Dataset.zip( (dataset1, dataset2) )
        print( dataset3.element_spec )

        # Dataset containing a sparse tensor.
        dataset4 = tf.data.Dataset.from_tensors(
            tf.SparseTensor( indices=[[0, 0], [1, 2]], values=[1, 2], dense_shape=[3, 4] )
        )
        print( dataset4.element_spec )

        # Dataset.element_spec.value_type: see the type of value represented by the element spec
        print( dataset4.element_spec.value_type )


        # Dataset transformations: Dataset.map(), Dataset.filter()
        #   apply a function to each element
        #     element structure determines the arguments of the function

        dataset1 = tf.data.Dataset.from_tensor_slices(
            tf.random.uniform( [4, 10], minval=1, maxval=10, dtype=tf.int32 )
        )
        print( dataset1 )

        for z in dataset1:
            print( z.numpy() )

        dataset2 = tf.data.Dataset.from_tensor_slices(
            (
                tf.random.uniform( [4] ),
                tf.random.uniform( [4, 100], maxval=100, dtype=tf.int32 )
            )
        )
        print( dataset2 )

        dataset3 = tf.data.Dataset.zip( (dataset1, dataset2) )
        print( dataset3 )

        for a, (b,c) in dataset3:
            print( f"shapes: {a.shape}, {b.shape}, {c.shape}")
    

    
    def reading_input_data():
        ###
        # **Consuming NumPy Arrays
        # Simplest way to create a Dataset from input data that fits in memory
        #   convert them to tf.Tensor objects and use Dataset.from_tensor_slices()
        
        # Example
        #   Note: features/labels arrays are embedded in TensorFlow graph as tf.constant() operations
        #     works well for a small dataset
        #     wastes memory because the contents of the array will be copied multiple times
        #       can run into the 2GB limit for the tf.GraphDef protocol buffer.
        train, test = tf.keras.datasets.fashion_mnist.load_data()
        images, labels = train
        images = images/255
        dataset = tf.data.Dataset.from_tensor_slices( (images, labels) )
        print( dataset )

        ###
        # **Consuming Python Generators
        # convienient approach
        # limited portability and scalibility
        #   must run in the same python process that created the generator
        #   still subject to the Python GIL
        #     prevents multithreaded CPython programs from taking full advantage of multiprocessor systems

        # Example
        def count(stop):
            i = 0
            while i < stop:
                yield i
                i += 1

        for n in count(5):
            print( n )
        
        # Dataset.from_generator constructor 
        #   converts python generator to tf.data.Dataset.
        #   input: 
        #     callable, not an iterator
        #       allows it to restart the generator when it reaches the end
        #     args argument (optional): 
        #       passed as the callable's arguments
        #     output_types argument (required): 
        #       tf.data builds a tf.Graph internally, and graph edges require a tf.dtype
        #     output_shapes argument (optionial but highly recomended)
        #       many tensorflow operations do not support tensors with unknown rank. 
        #         If the length of a particular axis is unknown or variable, set it as None in the output_shapes
        ds_counter = tf.data.Dataset.from_generator(
            count, 
            args=[25], 
            output_types=tf.int32, 
            output_shapes = (), 
        )
        for count_batch in ds_counter.repeat().batch(10).take(10):
            print( count_batch.numpy() )

        # example generator that demonstrates:
        #   return tuples of arrays, where the second array is a vector with unknown length.
        def gen_series():
            i = 0
            while True:
                size = np.random.randint( 0, 10 )
                yield i, np.random.normal( size=(size,) )
                i += 1
        for i, series in gen_series():
            print( i, ":", str(series) )
            if i > 5:
                break
        ds_series = tf.data.Dataset.from_generator(
            gen_series, 
            output_types=( tf.int32, tf.float32 ), 
            output_shapes=( (), (None,) )
        )
        print( ds_series )

        # when batching a dataset with variable shape, you need to use Dataset.padded_batch
        ds_series_batch = ds_series.shuffle( 20 ).padded_batch( 10 )
        ids, sequence_batch = next( iter(ds_series_batch) )
        print( ids.numpy() )
        print()
        print( sequence_batch.numpy() )

        # example: wrapping preprocessing.image.ImageDataGenerator as a tf.data.Dataset
        flowers = tf.keras.utils.get_file(
            'flower_photos',
            'https://storage.googleapis.com/download.tensorflow.org/example_images/flower_photos.tgz',
            untar=True
        )
        img_gen = tf.keras.preprocessing.image.ImageDataGenerator( rescale=1./255, rotation_range=20 )
        images, labels = next( img_gen.flow_from_directory(flowers) )
        print( images.dtype, images.shape )
        print( labels.dtype, labels.shape )

        ds = tf.data.Dataset.from_generator(
            lambda: img_gen.flow_from_directory( flowers ), 
            output_types=( tf.float32, tf.float32 ), 
            output_shapes=( [32,256,256,3], [32,5] )
        )
        print( ds.element_spec )

        for images, label in ds.take( 1 ):
            print( 'images.shape: ', images.shape )
            print( 'labels.shape: ', labels.shape )



        ###
        # **Consuming TFRecord data
        # tf.data API supports a variety of file formats so that you can process large datasets that do not fit in memory. 
        #   example: TFRecord file format
        #     simple record-oriented binary format that many TensorFlow applications use for training data.
        #     tf.data.TFRecordDataset class enables you to stream over the contents of one or more TFRecord files as part of an input pipeline.
        
        # example: using the test file from the French Street Name Signs (FSNS).
        # Creates a dataset that reads all of the examples from two files.
        fsns_test_file = tf.keras.utils.get_file(
            "fsns.tfrec", 
            "https://storage.googleapis.com/download.tensorflow.org/data/fsns-20160927/testdata/fsns-00000-of-00001"
        )

        # TFRecordDataset initializer 
        #   filenames argument 
        #     can be string, list of strings, or tf.Tensor of strings. 
        #       if two sets of files for training and validation purposes: create a factory method that produces the dataset, taking filenames as an input argument:
        dataset = tf.data.TFRecordDataset( filenames = [fsns_test_file] )
        print( dataset )

        # Many TensorFlow projects use serialized tf.train.Example records in their TFRecord files. 
        #   These need to be decoded before they can be inspected:
        raw_example = next( iter(dataset) )
        parsed = tf.train.Example.FromString( raw_example.numpy() )
        print( parsed.features.feature['image/text'] )

        ###
        # **Consuming text data
        # Many datasets are distributed as one or more text files. 
        # tf.data.TextLineDataset 
        #   input: one or more filenames 
        #   output: one string-valued element per line of those files.
        directory_url = 'https://storage.googleapis.com/download.tensorflow.org/data/illiad/'
        file_names = ['cowper.txt', 'derby.txt', 'butler.txt']
        file_paths = [
            tf.keras.utils.get_file(
                file_name, 
                directory_url + file_name
            )
            for file_name in file_names
        ]
        dataset = tf.data.TextLineDataset( file_paths )

        # Here are the first few lines of the first file:
        for line in dataset.take( 5 ):
            print( line.numpy() )
        
        # Dataset.interleave.
        #   alternate lines between files
        #   makes it easier to shuffle files together. 
        # Here are the first, second and third lines from each translation:
        files_ds = tf.data.Dataset.from_tensor_slices( file_paths )
        lines_ds = files_ds.interleave( tf.data.TextLineDataset, cycle_length=3 )

        for i, line in enumerate( lines_ds.take(9) ):
            if i % 3 == 0:
                print()
            print( line.numpy() )
        
        # By default, a TextLineDataset yields every line of each file, which may not be desirable, 
        # for example, if the file starts with a header line, or contains comments. 
        # These lines can be removed using the Dataset.skip() or Dataset.filter() transformations. 
        # Here, you skip the first line, then filter to find only survivors.
        titanic_file = tf.keras.utils.get_file(
            "train.csv", 
            "https://storage.googleapis.com/tf-datasets/titanic/train.csv"
        )
        titanic_lines = tf.data.TextLineDataset( titanic_file )
        for line in titanic_lines.take( 10 ):
            print( line.numpy() )
        
        def survived( line ):
            return tf.not_equal( tf.strings.substr(line, 0, 1), "0" )
        survivors = titanic_lines.skip( 1 ).filter( survived )
        for line in survivors.take( 10 ):
            print( line.numpy() )

        ###
        # **Consuming CSV data
        # The CSV file format is a popular format for storing tabular data in plain text

        # example:
        titanic_file = tf.keras.utils.get_file(
            "train.csv", 
            "https://storage.googleapis.com/tf-datasets/titanic/train.csv"
        )
        df = pd.read_csv( titanic_file )
        print( df.head() )

        # If data fits in memory
        #   use Dataset.from_tensor_slices method on dictionaries
        titanic_slices = tf.data.Dataset.from_tensor_slices( dict(df) )
        for feature_batch in titanic_slices.take( 1 ):
            for key, value in feature_batch.items():
                print( "  {!r:20s}: {}".format(key, value) )
        
        # load from disk as necessary.
        #   The tf.data module provides methods to extract records from one or more CSV files that comply with RFC 4180.
        #   The experimental.make_csv_dataset function is the high level interface for reading sets of csv files. 
        #   It supports column type inference and many other features, like batching and shuffling, to make usage simple.
        titanic_batches = tf.data.experimental.make_csv_dataset(
            titanic_file, 
            batch_size=4,
            label_name="survived"
        )
        for feature_batch, label_batch in titanic_batches.take( 1 ):
            print( "'survived': {}".format(label_batch) )
            print( "features:" )
            for key, value in feature_batch.items():
                print( "  {!r:20s}: {}".format(key, value) )
        
        # select_columns argument: subset of columns
        titanic_batches = tf.data.experimental.make_csv_dataset(
            titanic_file, 
            batch_size=4,
            label_name="survived", 
            select_columns=['class', 'fare', 'survived']
        )
        for feature_batch, label_batch in titanic_batches.take( 1 ):
            print( "'survived': {}".format(label_batch) )
            for key, value in feature_batch.items():
                print( "  {!r:20s}: {}".format(key, value) )
        
        # lower-level experimental.CsvDataset class 
        #   provides finer grained control. 
        #   does not support column type inference. 
        #       Instead you must specify the type of each column. 
        titanic_types  = [ tf.int32, tf.string, tf.float32, tf.int32, tf.int32, tf.float32, tf.string, tf.string, tf.string, tf.string ] 
        dataset = tf.data.experimental.CsvDataset(
            titanic_file, 
            titanic_types , 
            header=True
        )
        for line in dataset.take( 10 ):
            print( [item.numpy() for item in line] )
        
        # If some columns are empty, 
        #   this low-level interface allows you to provide default values instead of column types.
        # example: dataset from CSV files with columns which may have missing values.
        '''record_defaults = [ 999, 999, 999, 999 ]
        dataset = tf.data.experimental.CsvDataset( "missing.csv", record_defaults )
        dataset = dataset.map( lambda *items: tf.stack(items) )
        print( dataset )
        for line in dataset:
            print( line.numpy() )'''

        # CsvDataset
        #   header argument
        #     ignore header line
        #   select_cols argument
        #     ignore columns
        # example: dataset from CSV files with headers, extracting float data from columns 2 and 4.
        '''record_defaults = [ 999, 999 ] # Only provide defaults for the selected columns
        dataset = tf.data.experimental.CsvDataset(
            "missing.csv", 
            record_defaults, 
            select_cols=[1, 3]
        )
        dataset = dataset.map( lambda *items: tf.stack(items) )
        print( dataset )
        for line in dataset:
            print( line.numpy() )'''


        ###
        # **Consuming sets of files
        # many datasets are distributed as a set of files
        #   where each file is an example.
        flowers_root = tf.keras.utils.get_file(
            'flower_photos',
            'https://storage.googleapis.com/download.tensorflow.org/example_images/flower_photos.tgz',
            untar=True
        )
        flowers_root = pathlib.Path( flowers_root )
        
        # The root directory contains a directory for each class:
        for item in flowers_root.glob( "*" ):
            print( item.name )

        # The files in each class directory are examples:
        list_ds = tf.data.Dataset.list_files( str(flowers_root/'*/*') )
        for f in list_ds.take( 5 ):
            print( f.numpy() )

        # Read the data using the tf.io.read_file function
        # extract the label from the path, returning (image, label) pairs:
        def process_path( file_path ):
            label = tf.strings.split( file_path, os.sep )[ -2 ]
            return tf.io.read_file( file_path ), label
        labeled_ds = list_ds.map( process_path )
        for image_raw, label_text in labeled_ds.take( 1 ):
            print( repr(image_raw.numpy()[:100]) )
            print()
            print( label_text.numpy() )

    
    def batching_dataset_elements():
        ###
        # **Simple batching
        # The simplest form of batching 
        #   stacks n consecutive elements of a dataset into a single element. 
        #   The Dataset.batch() transformation does exactly this, 
        #     with the same constraints as the tf.stack() operator, applied to each component of the elements: i.e. 
        #       for each component i, all elements must have a tensor of the exact same shape.
        inc_dataset = tf.data.Dataset.range( 100 )
        dec_dataset = tf.data.Dataset.range( 0, -100, -1 )
        dataset = tf.data.Dataset.zip( (inc_dataset, dec_dataset) )
        batched_dataset = dataset.batch( 4 )
        for batch in batched_dataset.take( 4 ):
            print( [arr.numpy() for arr in batch] )

        # While tf.data tries to propagate shape information, 
        # the default settings of Dataset.batch result in an unknown batch size 
        #   because the last batch may not be full. Note the Nones in the shape:
        print( batched_dataset )

        # Use the drop_remainder argument to ignore that last batch, and get full shape propagation:
        batched_dataset = dataset.batch( 7, drop_remainder=True )  
        print( batched_dataset )

        ###
        # **Batching tensors with padding
        # The above recipe works for tensors that all have the same size. 
        # However, many models (e.g. sequence models) work with input data that can have varying size 
        #   (e.g. sequences of different lengths). 
        # To handle this case, the Dataset.padded_batch transformation 
        #   batch tensors of different shape by specifying one or more dimensions in which they may be padded.
        dataset = tf.data.Dataset.range( 100 )
        dataset = dataset.map( lambda x: tf.fill([tf.cast(x, tf.int32)], x) )
        dataset = dataset.padded_batch( 4, padded_shapes=(None,) )

        for batch in dataset.take( 2 ):
            print( batch.numpy() )
            print()

        # The Dataset.padded_batch transformation allows you to 
        #   set different padding for each dimension of each component, 
        #   and it may be variable-length (signified by None in the example above) or constant-length. 
        #   It is also possible to override the padding value, which defaults to 0.

    def training_workflows():
        ###
        # *Processing multiple epochs

        # The tf.data API offers two main ways to process multiple epochs of the same data.
        # The simplest way to iterate over a dataset in multiple epochs is to use the Dataset.repeat() transformation. 
        
        # First, create a dataset of titanic data:
        titanic_file = tf.keras.utils.get_file(
            "train.csv", 
            "https://storage.googleapis.com/tf-datasets/titanic/train.csv"
        )
        titanic_lines = tf.data.TextLineDataset( titanic_file )
        def plot_batch_sizes( ds ):
            batch_sizes = [ batch.shape[0] for batch in ds ]
            plt.bar( range(len(batch_sizes)), batch_sizes )
            plt.xlabel( 'Batch number' )
            plt.ylabel( 'Batch size' )
            plt.show()

        # Applying the Dataset.repeat() transformation with no arguments will repeat the input indefinitely.
        # The Dataset.repeat transformation concatenates its arguments without signaling the end of one epoch and the beginning of the next epoch. 
        # Because of this a Dataset.batch applied after Dataset.repeat will yield batches that straddle epoch boundaries:
        titanic_batches = titanic_lines.repeat( 3 ).batch( 128 )
        plot_batch_sizes( titanic_batches )

        # If you need clear epoch separation, put Dataset.batch before the repeat:
        titanic_batches = titanic_lines.batch( 128 ).repeat( 3 )
        plot_batch_sizes( titanic_batches )

        # If you would like to perform a custom computation (e.g. to collect statistics) at the end of each epoch 
        #   then it's simplest to restart the dataset iteration on each epoch:
        epochs = 3
        dataset = titanic_lines.batch( 128 )

        for epoch in range( epochs ):
            for batch in dataset:
                print( batch.shape )
            print( "End of epoch: ", epoch )

        ###
        # *Randomly shuffling input data
        # The Dataset.shuffle() transformation 
        #   maintains a fixed-size buffer and 
        #   chooses the next element uniformly at random from that buffer.
        #   Note: While large buffer_sizes 
        #     shuffle more thoroughly, 
        #     they can take a lot of memory, and significant time to fill. 
        #     Consider using Dataset.interleave across files if this becomes a problem.

        # Add an index to the dataset so you can see the effect:
        lines = tf.data.TextLineDataset( titanic_file )
        counter = tf.data.experimental.Counter()

        dataset = tf.data.Dataset.zip( (counter, lines) )
        dataset = dataset.shuffle( buffer_size=100 )
        dataset = dataset.batch( 20 )
        print( dataset )

        # Since the buffer_size is 100, and the batch size is 20, 
        #   the first batch contains no elements with an index over 120.
        n, line_batch = next( iter(dataset) )
        print( n.numpy() )

        # As with Dataset.batch the order relative to Dataset.repeat matters.
        # Dataset.shuffle doesn't signal the end of an epoch until the shuffle buffer is empty. 
        # So a shuffle placed before a repeat will show every element of one epoch before moving to the next:

        dataset = tf.data.Dataset.zip( (counter, lines) )
        shuffled = dataset.shuffle( buffer_size=100 ).batch( 10 ).repeat( 2 )
        print( "Here are the item ID's near the epoch boundary:\n" )
        for n, line_batch in shuffled.skip( 60 ).take( 5 ):
            print( n.numpy() )

        shuffle_repeat = [ n.numpy().mean() for n, line_batch in shuffled ]
        plt.plot( shuffle_repeat, label="shuffle().repeat()" )
        plt.ylabel( "Mean item ID" )
        plt.legend()
        plt.show()

        # But a repeat before a shuffle mixes the epoch boundaries together:
        dataset = tf.data.Dataset.zip( (counter, lines) )
        shuffled = dataset.repeat( 2 ).shuffle( buffer_size=100 ).batch( 10 )
        print( "Here are the item ID's near the epoch boundary:\n" )
        for n, line_batch in shuffled.skip( 55 ).take( 15 ):
            print( n.numpy() )

        repeat_shuffle = [ n.numpy().mean() for n, line_batch in shuffled ]
        plt.plot( shuffle_repeat, label="shuffle().repeat()" )
        plt.plot( repeat_shuffle, label="repeat().shuffle()" )
        plt.ylabel( "Mean item ID" )
        plt.legend()
        plt.show()


    def preprocessing_data():

        # Dataset.map(f) transformation 
        #   produces a new dataset 
        #     by applying a given function f to each element of the input dataset. 
        #   It is based on the map() function that is commonly applied to lists (and other structures) in functional programming languages. 
        #   The function f 
        #     takes the tf.Tensor objects that represent a single element in the input, and 
        #     returns the tf.Tensor objects that will represent a single element in the new dataset. 
        #     Its implementation uses standard TensorFlow operations to transform one element into another.
        # This section covers common examples of how to use Dataset.map().

        ###
        # *Decoding image data and resizing it
        # When training a neural network on real-world image data, 
        #   it is often necessary to convert images of different sizes to a common size, 
        #     so that they may be batched into a fixed size.
        
        # Rebuild the flower filenames dataset:
        flowers_root = tf.keras.utils.get_file(
            'flower_photos',
            'https://storage.googleapis.com/download.tensorflow.org/example_images/flower_photos.tgz',
            untar=True
        )
        flowers_root = pathlib.Path( flowers_root )
        list_ds = tf.data.Dataset.list_files( str(flowers_root/'*/*') )

        # Write a function that manipulates the dataset elements.
        #   Reads an image from a file, 
        #   decodes it into a dense tensor, 
        #   and resizes it to a fixed shape.
        def parse_image( filename ):
            parts = tf.strings.split( filename, os.sep )
            label = parts[ -2 ]

            image = tf.io.read_file( filename )
            image = tf.image.decode_jpeg( image )
            image = tf.image.convert_image_dtype( image, tf.float32 )
            image = tf.image.resize( image, [128, 128] )
            return image, label

        # Test that it works.
        file_path = next( iter(list_ds) )
        image, label = parse_image( file_path )
        
        def show( image, label ):
            plt.figure()
            plt.imshow( image )
            plt.title( label.numpy().decode('utf-8') )
            plt.axis( 'off' )
            plt.show()

        show( image, label )

        # Map it over the dataset.

        images_ds = list_ds.map( parse_image )

        for image, label in images_ds.take( 2 ):
            show( image, label )


        ###
        # *Applying arbitrary Python logic

        # For performance reasons, 
        #   use TensorFlow operations for preprocessing your data whenever possible. 
        #   However, it is sometimes useful to call external Python libraries when parsing your input data. 
        # You can use the tf.py_function() operation in a Dataset.map() transformation.

        # For example, if you want to apply a random rotation, 
        #   the tf.image module only has tf.image.rot90, which is not very useful for image augmentation.
        #     Note: tensorflow_addons has a TensorFlow compatible rotate in tensorflow_addons.image.rotate.

        # To demonstrate tf.py_function, 
        #   try using the scipy.ndimage.rotate function instead:

        import scipy.ndimage as ndimage

        def random_rotate_image( image ):
            image = ndimage.rotate(
                image, 
                np.random.uniform(-30, 30), 
                reshape=False
            )
            return image

        image, label = next( iter(images_ds) )
        image = random_rotate_image( image )
        show( image, label )

        # To use this function with Dataset.map the same caveats apply as with Dataset.from_generator, 
        # you need to describe the return shapes and types when you apply the function:

        def tf_random_rotate_image( image, label ):
            im_shape = image.shape
            [ image, ] = tf.py_function( random_rotate_image, [image], [tf.float32] )
            image.set_shape( im_shape ) 
            return image, label

        rot_ds = images_ds.map( tf_random_rotate_image )

        for image, label in rot_ds.take( 2 ):
            show( image, label )

        ###
        # *Parsing tf.Example protocol buffer messages

        # Many input pipelines extract tf.train.Example protocol buffer messages from a TFRecord format. 
        #   Each tf.train.Example record contains one or more "features", 
        #     and the input pipeline typically converts these features into tensors.
        fsns_test_file = tf.keras.utils.get_file(
            "fsns.tfrec", 
            "https://storage.googleapis.com/download.tensorflow.org/data/fsns-20160927/testdata/fsns-00000-of-00001"
        )
        dataset = tf.data.TFRecordDataset( filenames = [fsns_test_file] )
        print( dataset )

        # You can work with tf.train.Example protos outside of a tf.data.Dataset to understand the data:
        raw_example = next( iter(dataset) )
        parsed = tf.train.Example.FromString( raw_example.numpy() )

        feature = parsed.features.feature
        raw_img = feature[ 'image/encoded' ].bytes_list.value[ 0 ]
        img = tf.image.decode_png( raw_img )
        plt.imshow( img )
        plt.axis( 'off' )
        _ = plt.title( feature["image/text"].bytes_list.value[0] )

        raw_example = next( iter(dataset) )

        def tf_parse( eg ):
            example = tf.io.parse_example(
                eg[ tf.newaxis ], 
                {
                    'image/encoded': tf.io.FixedLenFeature( shape=(), dtype=tf.string ),
                    'image/text': tf.io.FixedLenFeature( shape=(), dtype=tf.string )
                }
            )
            return example[ 'image/encoded' ][ 0 ], example[ 'image/text' ][ 0 ]

        img, txt = tf_parse( raw_example )
        print( txt.numpy() )
        print( repr(img.numpy()[:20]), "..." )

        decoded = dataset.map( tf_parse )
        print( decoded )

        image_batch, text_batch = next( iter(decoded.batch(10)) )
        print( image_batch.shape )


        ###
        # *Time series windowing
        # Time series data is often organized with the time axis intact.
        # Use a simple Dataset.range to demonstrate:
        range_ds = tf.data.Dataset.range( 100000 )
        
        # Typically, models based on this sort of data will want a contiguous time slice.
        # The simplest approach would be to batch the data:
        # Using batch
        batches = range_ds.batch( 10, drop_remainder=True )
        for batch in batches.take( 5 ):
            print( batch.numpy() )
        
        # Or to make dense predictions one step into the future, 
        #   you might shift the features and labels by one step relative to each other:

        def dense_1_step( batch ):
            # Shift features and labels one step relative to each other.
            return batch[:-1], batch[1:]

        predict_dense_1_step = batches.map( dense_1_step )

        for features, label in predict_dense_1_step.take( 3 ):
            print( features.numpy(), " => ", label.numpy() )

        # To predict a whole window instead of a fixed offset you can split the batches into two parts:
        batches = range_ds.batch( 15, drop_remainder=True )

        def label_next_5_steps( batch ):
            return (
                batch[:-5],   # Inputs: All except the last 5 steps
                batch[-5:]    # Labels: The last 5 steps
                )   

        predict_5_steps = batches.map( label_next_5_steps )

        for features, label in predict_5_steps.take( 3 ):
            print( features.numpy(), " => ", label.numpy() )
        
        # To allow some overlap between the features of one batch and the labels of another, use Dataset.zip:
        feature_length = 10
        label_length = 3

        features = range_ds.batch( feature_length, drop_remainder=True )
        labels = range_ds.batch( feature_length ).skip( 1 ).map( lambda labels: labels[:label_length] )

        predicted_steps = tf.data.Dataset.zip( (features, labels) )

        for features, label in predicted_steps.take( 5 ):
            print( features.numpy(), " => ", label.numpy() )
        
        # Using window
        # While using Dataset.batch works, there are situations where you may need finer control. 
        # The Dataset.window method gives you complete control, 
        # but requires some care: it returns a Dataset of Datasets. See Dataset structure for details.

        window_size = 5

        windows = range_ds.window( window_size, shift=1 )
        for sub_ds in windows.take( 5 ):
            print( sub_ds )

        # The Dataset.flat_map method can take a dataset of datasets and flatten it into a single dataset:

        for x in windows.flat_map( lambda x: x ).take( 30 ):
            print( x.numpy(), end=' ' )

        # In nearly all cases, you will want to .batch the dataset first:

        def sub_to_batch( sub ):
            return sub.batch( window_size, drop_remainder=True )

        for example in windows.flat_map( sub_to_batch ).take( 5 ):
            print( example.numpy() )

        # Now, you can see that the shift argument controls how much each window moves over.
        # Putting this together you might write this function:

        def make_window_dataset( ds, window_size=5, shift=1, stride=1 ):
            windows = ds.window(
                window_size, 
                shift=shift, 
                stride=stride
            )

            def sub_to_batch( sub ):
                return sub.batch( window_size, drop_remainder=True )

            windows = windows.flat_map( sub_to_batch )
            return windows

        ds = make_window_dataset(
            range_ds, 
            window_size=10, 
            shift=5, 
            stride=3
        )

        for example in ds.take( 10 ):
            print( example.numpy() )
        
        # Then it's easy to extract labels, as before:

        dense_labels_ds = ds.map( dense_1_step )

        for inputs, labels in dense_labels_ds.take( 3 ):
            print( inputs.numpy(), "=>", labels.numpy() )


        ###
        # *Resampling
        # When working with a dataset that is very class-imbalanced, 
        #   you may want to resample the dataset. 
        # tf.data provides two methods to do this. 
        # The credit card fraud dataset is a good example of this sort of problem.
        zip_path = tf.keras.utils.get_file(
            origin='https://storage.googleapis.com/download.tensorflow.org/data/creditcard.zip',
            fname='creditcard.zip',
            extract=True
        )
        csv_path = zip_path.replace('.zip', '.csv')

        creditcard_ds = tf.data.experimental.make_csv_dataset(
            csv_path, 
            batch_size=1024, 
            label_name="Class",
            column_defaults=[ float() ]*30 + [ int() ] # Set the column types: 30 floats and an int.
        )

        # Now, check the distribution of classes, it is highly skewed:

        def count( counts, batch ):
            features, labels = batch
            class_1 = labels == 1
            class_1 = tf.cast( class_1, tf.int32 )

            class_0 = labels == 0
            class_0 = tf.cast( class_0, tf.int32 )

            counts[ 'class_0' ] += tf.reduce_sum( class_0 )
            counts[ 'class_1' ] += tf.reduce_sum( class_1 )

            return counts

        counts = creditcard_ds.take( 10 ).reduce(
            initial_state={ 'class_0': 0, 'class_1': 0 },
            reduce_func = count
        )

        counts = np.array(
            [ counts['class_0'].numpy(), counts['class_1'].numpy() ]
        ).astype( np.float32 )

        fractions = counts / counts.sum()
        print( fractions )

        # A common approach to training with an imbalanced dataset is to balance it. 
        # tf.data includes a few methods which enable this workflow:
        # Datasets sampling

        # One approach to resampling a dataset is to use sample_from_datasets. 
        # This is more applicable when you have a separate data.Dataset for each class.

        # Here, just use filter to generate them from the credit card fraud data:

        negative_ds = (
            creditcard_ds
                .unbatch()
                .filter( lambda features, label: label==0 )
                .repeat()
        )

        positive_ds = (
            creditcard_ds
                .unbatch()
                .filter( lambda features, label: label==1 )
                .repeat()
        )

        for features, label in positive_ds.batch( 10 ).take( 1 ):
            print( label.numpy() )

        # To use tf.data.experimental.sample_from_datasets pass the datasets, and the weight for each:

        balanced_ds = tf.data.experimental.sample_from_datasets(
            [ negative_ds, positive_ds ], 
            [ 0.5, 0.5 ]
        ).batch( 10 )

        # Now the dataset produces examples of each class with 50/50 probability:
        for features, labels in balanced_ds.take( 10 ):
            print( labels.numpy() )

        # Rejection resampling
        # One problem with the above experimental.sample_from_datasets approach is that 
        #   it needs a separate tf.data.Dataset per class. 
        #     Using Dataset.filter works, but results in all the data being loaded twice.
        # The data.experimental.rejection_resample function can be applied to a dataset to rebalance it, 
        #   while only loading it once. 
        #   Elements will be dropped from the dataset to achieve balance.
        # data.experimental.rejection_resample takes a class_func argument. 
        #   This class_func is applied to each dataset element, 
        #   and is used to determine which class an example belongs to for the purposes of balancing.

        # The elements of creditcard_ds are already (features, label) pairs. 
        # So the class_func just needs to return those labels:

        def class_func( features, label ):
            return label

        # The resampler also needs a target distribution, and 
        #   optionally an initial distribution estimate:

        resampler = tf.data.experimental.rejection_resample(
            class_func, 
            target_dist=[ 0.5, 0.5 ], 
            initial_dist=fractions
        )

        # The resampler deals with individual examples, 
        #   so you must unbatch the dataset before applying the resampler:

        resample_ds = creditcard_ds.unbatch().apply( resampler ).batch( 10 )

        # The resampler returns creates (class, example) pairs from the output of the class_func. 
        # In this case, the example was already a (feature, label) pair, so use map to drop the extra copy of the labels:

        balanced_ds = resample_ds.map(
            lambda extra_label, features_and_label: features_and_label
        )

        # Now the dataset produces examples of each class with 50/50 probability:

        for features, labels in balanced_ds.take( 10 ):
            print( labels.numpy() )



    def iterator_checkpointing():

        # Tensorflow supports taking checkpoints 
        #   so that when your training process restarts it can restore the latest checkpoint to recover most of its progress. 
        # In addition to checkpointing the model variables, 
        #   you can also checkpoint the progress of the dataset iterator. 
        #     This could be useful if you have a large dataset and don't want to start the dataset from the beginning on each restart. 
        #     Note however that iterator checkpoints may be large, since transformations such as shuffle and prefetch require buffering elements within the iterator.

        # To include your iterator in a checkpoint, pass the iterator to the tf.train.Checkpoint constructor.

        range_ds = tf.data.Dataset.range( 20 )
        
        iterator = iter( range_ds )
        ckpt = tf.train.Checkpoint( 
            step=tf.Variable( 0 ), 
            iterator=iterator
        )
        manager = tf.train.CheckpointManager(
            ckpt, 
            '/tmp/my_ckpt', 
            max_to_keep=3
        )

        print( [next(iterator).numpy() for _ in range(5)] )

        save_path = manager.save()

        print( [next(iterator).numpy() for _ in range(5)] )

        ckpt.restore( manager.latest_checkpoint )

        print( [next(iterator).numpy() for _ in range(5)] )

        # Note: It is not possible to checkpoint an iterator 
        #   which relies on external state such as a tf.py_function. 
        #   Attempting to do so will raise an exception complaining about the external state.



    def using_tf_data_with_tf_keras():
        # The tf.keras API simplifies many aspects of creating and executing machine learning models. 
        # Its .fit() and .evaluate() and .predict() APIs support datasets as inputs. 
        # Here is a quick dataset and model setup:

        train, test = tf.keras.datasets.fashion_mnist.load_data()

        images, labels = train
        images = images/255.0
        labels = labels.astype( np.int32 )

        fmnist_train_ds = tf.data.Dataset.from_tensor_slices( (images, labels) )
        fmnist_train_ds = fmnist_train_ds.shuffle( 5000 ).batch( 32 )

        model = tf.keras.Sequential(
            [ tf.keras.layers.Flatten(), tf.keras.layers.Dense(10) ]
        )

        model.compile(
            optimizer='adam',
            loss=tf.keras.losses.SparseCategoricalCrossentropy( from_logits=True ), 
            metrics=['accuracy']
        )

        # Passing a dataset of (feature, label) pairs is all that's needed for Model.fit and Model.evaluate:
        model.fit( fmnist_train_ds, epochs=2 )

        # If you pass an infinite dataset, for example by calling Dataset.repeat(), 
        #   you just need to also pass the steps_per_epoch argument:
        model.fit( 
            fmnist_train_ds.repeat(), 
            epochs=2, 
            steps_per_epoch=20
        )

        # For evaluation you can pass the number of evaluation steps:
        loss, accuracy = model.evaluate( fmnist_train_ds )
        print( "Loss :", loss )
        print( "Accuracy :", accuracy )

        # For long datasets, set the number of steps to evaluate:
        loss, accuracy = model.evaluate(
            fmnist_train_ds.repeat(), 
            steps=10
        )
        print( "Loss :", loss )
        print( "Accuracy :", accuracy )

        # The labels are not required in when calling Model.predict.
        predict_ds = tf.data.Dataset.from_tensor_slices( images ).batch( 32 )
        result = model.predict(
            predict_ds, 
            steps = 10
        )
        print( result.shape )

        # But the labels are ignored if you do pass a dataset containing them:
        result = model.predict(
            fmnist_train_ds, 
            steps = 10
        )
        print( result.shape )





















    basic_mechanics()
    reading_input_data()
    batching_dataset_elements()
    training_workflows()
    preprocessing_data()
    iterator_checkpointing()
    using_tf_data_with_tf_keras()






if __name__ == "__main__":

    data_input_pipelines_guide()















