function hw8(questioNum)
    % Running this file will download CIFAR10 and place the images into a
    % training folder and test folder in the current directory
    % These will be used for the three demos in this folder. 
    % Please note this will take a few minutes to run, but only needs to be run
    % once.
    % Copyright 2017 The MathWorks, Inc.
    % Download the CIFAR-10 dataset
    if ~exist('cifar-10-batches-mat','dir')
        cifar10Dataset = 'cifar-10-matlab';
        disp('Downloading 174MB CIFAR-10 dataset...');   
        websave([cifar10Dataset,'.tar.gz'],...
            ['https://www.cs.toronto.edu/~kriz/',cifar10Dataset,'.tar.gz']);
        gunzip([cifar10Dataset,'.tar.gz'])
        delete([cifar10Dataset,'.tar.gz'])
        untar([cifar10Dataset,'.tar'])
        delete([cifar10Dataset,'.tar'])
    end    

    % Prepare the CIFAR-10 dataset
    if ~exist('cifar10Train','dir')
        disp('Saving the Images in folders. This might take some time...');    
        saveCIFAR10AsFolderOfImages('cifar-10-batches-mat', pwd, true);
        disp('Done');
    end
    
    if questioNum==1
        
        % Load images
        categories = {'deer','dog','frog','cat','bird','horse'};
        rootFolder = 'cifar10Train';
        imds = imageDatastore(fullfile(rootFolder, categories), ...
            'LabelSource', 'foldernames');

        layers = Q1(); % Load model structure
        
        % Train network
        opts = trainingOptions('sgdm', ...
                                'ExecutionEnvironment','auto',...
                                'InitialLearnRate', 0.001, ...
                                'LearnRateSchedule', 'piecewise', ...
                                'LearnRateDropFactor', 0.1, ...
                                'LearnRateDropPeriod', 8, ...
                                'L2Regularization', 0.004, ...
                                'MaxEpochs', 10, ...
                                'MiniBatchSize', 50, ...
                                'Verbose', true);
       [net, info] = trainNetwork(imds, layers, opts);
       
       
        % Load test data
        rootFolder = 'cifar10Test';
        imds_test = imageDatastore(fullfile(rootFolder, categories), ...
                                    'LabelSource', 'foldernames');
                                
        % Classify test data using our network
        labels = classify(net, imds_test);

        % Plot a 4x4 grid of randomly selected test cases
        figure(1);
        set(gcf,'position',[200,200,500,500])
        for i=1:16
            subplot(4,4,i)
            ii = randi(size(categories,2)*1000); % randomly select a test case
            im = imread(imds_test.Files{ii});
            imshow(im); % display test case
            if labels(ii) == imds_test.Labels(ii)
               colorText = 'g'; % green label for correct prediction
            else
                colorText = 'r'; % red label for incorrect prediction
            end
            title(char(labels(ii)),'Color',colorText);
        end
        sgtitle('prediction');

       
       % This could take a while if you are not using a GPU
        confMat = confusionmat(imds_test.Labels, labels);
        confMat = confMat./sum(confMat,2);
        mean(diag(confMat))
       
    end
     
    
    if questioNum==2 

        % Load images
        rootFolder = 'cifar10Train';
        % categories = {'deer','dog','frog','cat','bird','horse'};
        categories = {'deer','dog','frog','cat'};
        imds = imageDatastore(fullfile(rootFolder, categories), 'LabelSource', 'foldernames');
        
        % Downsample dataset
        imds = splitEachLabel(imds, 500, 'randomize'); % we only need 500 images per class
        imds.ReadFcn = @readFunctionTrain;
        
        layers = Q2();  % Load model structure
        
        % Increase learning rate on last layer
        layers(end-2).WeightLearnRateFactor = 10;
        layers(end-2).WeightL2Factor = 1;
        layers(end-2).BiasLearnRateFactor = 20;
        layers(end-2).BiasL2Factor = 0;

        % Train model
        opts = trainingOptions('sgdm', ...
        'ExecutionEnvironment','auto',...
        'LearnRateSchedule', 'none',...
        'InitialLearnRate', .0001,... 
        'MaxEpochs', 20, ...
        'MiniBatchSize', 32);
        convnet = trainNetwork(imds, layers, opts);
        
        % Load test data
        rootFolder = 'cifar10Test';
        testDS = imageDatastore(fullfile(rootFolder, categories), 'LabelSource', 'foldernames');
        testDS.ReadFcn = @readFunctionTrain;
        
        % Classify test data
        [labels,err_test] = classify(convnet, testDS, 'MiniBatchSize', 64);
        
        % Plot a 4x4 grid of randomly selected test cases
         for i=1:16
            ii = randi(size(categories,2)*1000);
            subplot(4,4,i)
            im = imread(testDS.Files{ii});
            imshow(im);
            if labels(ii) == testDS.Labels(ii)
               colorText = 'g'; 
            else
                colorText = 'r';
            end
            title(char(labels(ii)),'Color',colorText);
         end
         sgtitle('prediction');
        
        confMat = confusionmat(testDS.Labels, labels);
        confMat = confMat./sum(confMat,2);
        mean(diag(confMat))       

    end
    
    if questioNum==3
        Q3();
    end
end