
% This code will run much faster if you install CUDA support
function layers = Q1()

    layers = [
            imageInputLayer([32 32 3]); % input images are 32x32x3
           
            convolution2dLayer(5,16,'Padding',2,'BiasLearnRateFactor',2);
            maxPooling2dLayer(3,'Stride',2);
            reluLayer();

            convolution2dLayer(5,32,'Padding',2,'BiasLearnRateFactor',2);
            batchNormalizationLayer;
            averagePooling2dLayer(3,'Stride',2);
            % maxPooling2dLayer(3,'Stride',2);
            reluLayer();
            
%             convolution2dLayer(5,64,'Padding',2,'BiasLearnRateFactor',2);
%             batchNormalizationLayer;
%             averagePooling2dLayer(3,'Stride',2);
%             % maxPooling2dLayer(3,'Stride',2);
%             reluLayer();
            
            fullyConnectedLayer(64,'BiasLearnRateFactor',2);
            reluLayer();
            
            fullyConnectedLayer(6,'BiasLearnRateFactor',2);

            softmaxLayer();
            classificationLayer();];
    
end
