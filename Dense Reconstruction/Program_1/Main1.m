%% Structure From Motion From Multiple Views
 
clear 
close all
clc

%% Overview
 
addpath('LIbrary');

 
% 
imageDir = fullfile('C:\Users\Admin\Desktop\3D image\sevenda');
imSet = imageSet(imageDir);
 
images = cell(1, imSet.Count);
count=0;
for i = 1:imSet.Count
    count=count+1;
    I = read(imSet, i);
%     image=imcrop(I,[290.507380073801 24.090405904059 358.966789667897 449.653136531365]);
%     imshow(image)
%     pause(0.6)
   images{i} = rgb2gray(I);
%   I = imresize(I,[512 400]);
%             thisfile = sprintf('frame_%04d.tif', count);
%           imwrite(I,thisfile)
end

title('Input Image Sequence');

 


load(fullfile(imageDir, 'cameraParams.mat'));
 

I=images{1};
 

 
% roi =[290.507380073801 24.090405904059 358.966789667897 449.653136531365];
%  roi =[38 23.7068965517242 587.068965517241 951.724137931035];
%  roi=[30.5 6.5 949 1004];
% roi=[329.5 71.5 505 612];
% roi=[18.5 6.5 210 221];
% roi=[277.5 14.5 381 416];
roi=[75.5 37.5 585 857];%%selection is done using imtool
prevPoints   = detectSURFFeatures(I, 'NumOctaves', 60, 'ROI', roi);
 
prevFeatures = extractFeatures(I, prevPoints, 'Upright', true,'SURFSize',128);
 
vSet = viewSet;

 
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ones(3),...
    'Location', [0 0 0]);

%% Add the Rest of the Views

for i = 2:numel(images)
 
    I=images{i};
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I, 'NumOctaves', 60, 'ROI', roi);
%       imshow(I); hold on;
%   plot(currPoints.selectStrongest(50));
% pause(0.5)
%         currPoints   = detectSURFFeatures(I, 'NumOctaves', 50);
%  currPoints=detectHarrisFeatures(I, 'MinQuality', 0.0001, 'ROI', roi);
    currFeatures = extractFeatures(I, currPoints, 'Upright', true,'SURFSize',128);    
    indexPairs = matchFeatures(prevFeatures, currFeatures,  'MatchThreshold', 99,...
        'MaxRatio', .999999999,  'Metric','SSD','Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
     
    [relativeOrient, relativeLoc, inlierIdx] = EstimateRelative(
        matchedPoints1, matchedPoints2, cameraParams);
    
 
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = prevOrientation * relativeOrient;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
 
    tracks = findTracks(vSet);

   
    camPoses = poses(vSet);

 
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
 
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end

 
camPoses = poses(vSet);
 

 
goodIdx = (reprojectionErrors < 5);
xyzPoints1 = xyzPoints(goodIdx, :);

 

 

 
I = images{1}; 

 
prevPoints = detectHarrisFeatures(I, 'MinQuality', 0.0001, 'ROI', roi);

 
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

 
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);
 
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

 
for i = 2:numel(images)
    % Read and undistort the current image.
    I = images{i}; 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% 
tracks = findTracks(vSet);

%
camPoses = poses(vSet);

% 
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

%
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);

%% Display Dense Reconstruction
%%



 load('LIbrary/meshcharacter.mat')
 %  
shapeEV = double(shapeEV);
shapePC = double(shapePC);
shapeMU = double(shapeMU);



 
load('LIbrary/modeledges.mat') 
Ef = facethreeedges( tl,Ev );

% Face.vertics=xyzPoints;
FACE.vertices=xyzPoints;
FACE.faces = tl;
ndims = 10;
 
w_initialprior=0.3;
 
icefniter=20;

PRAM.Ef = Ef;
PRAM.Ev = Ev;
 
PRAM.w1 = 0.45; 
PRAM.w2 = 0.15;

%% 


im   =      read(imSet, i);


edgeim = edge(rgb2gray(im),'canny',0.15);


bs = marking(im);

[detectededges,pointsclouds]=Ver_id(bs,im);
 
%%
a=max(tl);
max_value=a(1);
max_initial_value=max(size(FACE.vertices));
while max_value<max_initial_value
%% 

[b,R,t,s] = Single_fitting( detectededges,shapePC,shapeMU,shapeEV,ndims,pointsclouds,w_initialprior );
FACE.vertices=reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC,1)/3)';
FACE.faces = tl;
max_initial_value=max(size(FACE.vertices));


end

[b,R,t,s] = Curve_fitting(im,detectededges,pointsclouds,shapePC,shapeMU,shapeEV,PRAM.Ef,PRAM.Ev,tl,ndims, w_initialprior, PRAM.w1, PRAM.w2,icefniter);
FACE.vertices = reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC,1)/3)';
 
maxiter = 20;
iter = 0;
diff = 1;
eps = 1e-9;

[r,c]=find(edgeim);
r = size(edgeim,1)+1-r;

while (iter<maxiter) && (diff>eps)
    
    FACE.vertices=reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC,1)/3)';
    [ PRAM.VER_TRY ] = ignore_overlapping( FACE,PRAM.Ef,PRAM.Ev,R );

    X = reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC(:,1:ndims),1)/3);   
     
    x_edge = R*X(:,PRAM.VER_TRY);
    x_edge = x_edge(1:2,:);
    x_edge(1,:)=s.*(x_edge(1,:)+t(1));
    x_edge(2,:)=s.*(x_edge(2,:)+t(2));
   
    [idx,d] = knnsearch([c r],x_edge');
    
    %idx = rangesearch([c r],x_edge',0.3);
     sortedd=sort(d);
    threshold = sortedd(round(0.95*length(sortedd)));
    idx = idx(d<threshold);
    PRAM.VER_TRY = PRAM.VER_TRY(d<threshold);

    b0 = b;
    [ R,t,s,b ] = Opti_Hard( b0,detectededges,shapeEV,shapeMU,shapePC,R,t,s,r,c,pointsclouds,PRAM,tl,false );
    
    diff = norm(b0-b);
    disp(num2str(diff));
    iter = iter+1;
%     pause
end

 
[ R,t,s,b ] = Opti_Hard( b,detectededges,shapeEV,shapeMU,shapePC,R,t,s,r,c,pointsclouds,PRAM,tl,true );


% disp('Rendering final results...');
% FACE.vertices=reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC,1)/3)';
% figure; subplot(1,3,1); patch(FACE, 'FaceColor', [1 1 1], 'EdgeColor', 'none', 'FaceLighting', 'phong'); light; axis equal; axis off;
% subplot(1,3,2); imshow(rendering_face_nonlinear(FACE,im,R,t,s,false));
% subplot(1,3,3); imshow(rendering_face_nonlinear(FACE,im,R,t,s,true));
hho=FACE.vertices ;
figure;
helperPlotCameras(camPoses);


pcshow(hho, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 200);
% % pcshow(xyzPoints)

grid on;

%



title('Dense Reconstruction');

     I1  = read(imSet, 1);
%164824
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2), size(I1, 2), size(I1, 2), size(I1, 2)], round(currPoints(:,2)),round(currPoints(:, 1)));
Index_val=find(colorIdx>size(allColors,1));
 colorIdx(Index_val)=99;
color = allColors(colorIdx, :);
R2=(hho);
[ROW COL]=size(R2);
% 
figure
grid on
ptCloudCurrent=pointCloud(R2);


pcshow(ptCloudCurrent, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 120)

camorbit(0, -240);

[model, msz] = model_parameter();
new_tl=FACE.faces;
shape=FACE.vertices;

alpha = randn(msz.n_shape_dim, 1);
beta  = randn(msz.n_tex_dim, 1);
 
tex    = texture_creation( beta,  model.texMU,   model.texPC,   model.texEV );

load('LIbrary/textureinfo.mat')
 
rp     = camera_parameter_tuning;
rp.phi = 0.5;
rp.dir_light.dir = [0;1;1];
rp.dir_light.intens = 0.1*ones(3,1);



disp('Rendering final results...');
FACE.vertices=reshape(shapePC(:,1:ndims)*b+shapeMU,3,size(shapePC,1)/3)';

 figure; patch(FACE, 'FaceColor', [1 1 1], 'EdgeColor', 'none', 'FaceLighting', 'phong'); light; axis equal; axis off;
figure; imshow(rendering_face_nonlinear(FACE,im,R,t,s,false));
figure; imshow(rendering_face_nonlinear(FACE,im,R,t,s,true));

count=1;



 for i=1:53490
for j=1:3
newshape(count)=shape(i,j);
count=count+1;
end
end
NEW_shape=newshape';
 plywrite('newProg_head.ply', NEW_shape, tex, new_tl);
 
 
 face_Annoation(shape,tex, new_tl, rp);


 
