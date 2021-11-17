% Modify the network from Alex to classify the 4 categories images. 
% Install toolbox parallel computing to accelerate your training speed.
function layers = Q2()

    net = alexnet;
    layers = net.Layers;
    layers = layers(1:end-3);
    
    % layers(end+1) = fullyConnectedLayer(64,'BiasLearnRateFactor',2);
    % layers(end+1) = reluLayer();
    layers(end+1) = fullyConnectedLayer(4,'BiasLearnRateFactor',2);
    layers(end+1) = softmaxLayer();
    layers(end+1) = classificationLayer();
end