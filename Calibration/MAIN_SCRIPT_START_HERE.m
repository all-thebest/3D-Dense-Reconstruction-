
cdir = pwd;

%Adding the video reader to the path
addpath(cdir);
addpath(fullfile(cdir,'mmread'));
addpath(fullfile(cdir,'TOOLBOX_calib_mod'));
fname = fullfile(cdir,'data', '20151105_084420.mp4');
%fname = fullfile('..','..','data', '20151124_030845.mp4');
%
%convert from RGB -> XYZ, use Y as luma
%A = [.490 .310 .200; .177 .813 .011; .000 .010 .999];
A = eye(3);
 %Do the Resize once
 ccount = 1;
 
 fprintf('Reading Video Object\n');
 video_obj = mmread(fname);
 
 fprintf('Writing files to be used for calibration\n');
  for ii = 1:4:video_obj.nrFramesTotal
      video_obj.frames(ii).cdata=imresize(video_obj.frames(ii).cdata,.25);
      rgbf = video_obj.frames(ii).cdata;
      
      rgbf = squeeze(reshape(rgbf,[],1,3))';
      xyzf = (A * double(rgbf))';
      xyzf = uint8(reshape(xyzf,size(video_obj.frames(ii).cdata,1),size(video_obj.frames(ii).cdata,2),3));
      video_obj.frames(ii).cdata = xyzf;
      %imwrite(rgb2gray(xyzf),sprintf('../calib2/calt%d.bmp',ccount),'BMP');
      imwrite(xyzf,sprintf('%s/cal_imgs/orig_calt%d.bmp',cdir,ccount),'BMP');
      ccount=ccount+1;
  end
  fprintf('Playing Video\n');
peakThresh =20;
edgeThresh = 50;
last_peaks = [];

cornr = zeros(4,2);
last_origin = zeros(1,2);
cb_state=[];
for ii = 1:4:video_obj.nrFramesTotal
    xyzf = video_obj.frames(ii).cdata;
if(ii == 19)
   aa= 1; 
end
   [cb, cb_state, bw_out, roi_image] = cbedge(xyzf,cb_state);
   crns = zeros(size(xyzf(:,:,1)));
   
   for cc = 1:length(cb)
       crns(cb(cc,1),cb(cc,2))= 1;
   end
   
   crns = imdilate(crns,strel('disk',5));
    
   cc = uint8(repmat(crns*255,[1 1 3]));
   cc(:,:,1)= 0;
   
   imshow(xyzf .* uint8(~logical(cc)));
   text(cb(1,2)+10,cb(1,1), 'Origin','color','r')
   %TRI = delaunay(I,J);
   %triplot(TRI,I,J); hold on
   %imshow(bbw_im*255);
    drawnow;
   

end

cd(fullfile(cdir,'cal_imgs'));
fprintf('Demo Complete\n');
calib_gui

fprintf('1. ''Click Standard''\n');
fprintf('2. Click ''Image names'' and type ''orig_calt'', then select ''bmp''\n')
fprintf('3. Click ''Extract Grid Corners''\n');
fprintf('4. Click ''Calibration''\n');

