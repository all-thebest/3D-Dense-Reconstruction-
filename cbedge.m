function [cb, state, out_is_bw, roi_img] = cbedge(img,state,thresh)

if(nargin < 3)
   thresh = 25; 
end

if(isempty(state))
    state.last_origin = [];
    state.last_points = [];
end


ste = strel('disk',50);
ste2 = strel('disk',2);
if(size(img,3)==1)
    [im_out, peaks] = harris(img,1,thresh,4);%25
else
    [im_out, peaks] = harris(rgb2gray(img),1,thresh,4);%.5
end

outs = outliers(peaks,floor(length(peaks)*.05));
out_ind =  outs(:,1).*outs(:,2);
out_ind =~isnan(out_ind );
peaks = peaks(out_ind,:);

pxl = zeros(length(peaks),3);
pxl_std = zeros(1,length(peaks));
for jj = 1:length(peaks)

    pxl(jj,:) = [img(peaks(jj,1),peaks(jj,2),:)];
end

for jj = 1:length(peaks)
    pxl_std(jj) = std(pxl(jj,:));
end


pxl_ind = (pxl_std < 9);
peaks2=peaks(pxl_ind,:);%[];



bw_im = zeros(size(im_out));
ind = sub2ind(size(im_out),peaks2(:,1),peaks2(:,2));



bw_im(ind) = 1;


NN = length(peaks2);
dilate_factor = 5;
while(NN > 5)
    ste = strel('disk',dilate_factor);
    bw_im_mask = imdilate(bw_im,ste);
    [bw_im_mask_label, NN]= bwlabel(bw_im_mask);

    if(NN < 7)
        dilate_factor = floor(dilate_factor * 1.1);
    else
        dilate_factor = dilate_factor * 3;
    end
    
    if(NN <= 7)
        ara = zeros(1,NN);
        for ii = 1:NN
            ara(ii) = sum(bw_im_mask_label(:) == ii);
        end
        [v, ind] = max(ara);
        ara(ind) = [];
        s = sum(ara);
        if(v/s > 4.5)
            break;
        end
    end    
end
ara = zeros(1,NN);
for ii = 1:NN
    ara(ii) = sum(bw_im_mask_label(:) == ii);
end
[~,iind] = max(ara);
bw_im_mask_label(bw_im_mask_label~= iind) = 0;

bbw_old = zeros(size(im_out));
if( ~isempty(state.last_points))
    for ii=1:length(state.last_points)
        bbw_old(state.last_points(ii,1),state.last_points(ii,2))=1;
    end
end
%Now we have points at the corners of the checkerboard
bbw_im = imdilate(logical(bw_im_mask_label & bw_im),ste2);



[I,J]= ind2sub(size(bw_im),find(logical(bw_im_mask_label & bw_im) == true));



k = convhull(I,J);
%plot(I(k),J(k),'r-',I,J,'b*')
roi_img= roipoly(bbw_im,J(k),I(k));

area_per_square=sum(sum(1.*roi_img==1))/63;
size_of_square=sqrt(area_per_square);

is_bw = bwmorph(roi_img,'shrink',2) & logical(bw_im_mask_label & bw_im);
out_is_bw = imdilate(is_bw,strel('disk',1));

[I,J]= ind2sub(size(is_bw),find(logical(out_is_bw) == true));
k = convhull(I,J);
roi_img = roipoly(bbw_im,J(k),I(k));
cout = corner(uint8(roi_img),[],[],[],0.2);

%
if(length(cout) ~= 4)
   for ii = 1:length(cout)
       for jj = ii:length(cout)
           if(ii == jj)
               continue;
           end
           
           d(jj,ii) = norm(cout(ii,:)-cout(jj,:));
           if(d < 20)
               cout(jj,:) = [];
               if(length(cout) == 4)
                  break; 
               end
           end
       end
       if(length(cout) == 4)
           break;
       end
   end
end

dist = zeros(4,4);
for ii = 1:length(cout)
    for jj = 1:length(cout)
       
        if(ii == jj)
            continue;
        end    
        
        dist(jj,ii)=norm(cout(ii,:)-cout(jj,:));
        
    end
end

origin_m = zeros(size(is_bw));
crns = zeros(size(is_bw));

if(isempty(state.last_origin))
    
    %find point closest to top left
    nrm =zeros(1,length(cout));
    for ii = 1:length(cout)
        nrm(ii) = norm(cout(ii,:));
    end
    
    [~,mi] = min(nrm);
    origin = cout(mi,:);
    state.last_origin =origin;
    state.last_points = cout;
else
    d = (cout(1:4,:)-repmat(state.last_origin,4,1)).^2;
    d = sum(d');
    [~, mi] = min(d);
    origin = cout(mi,:);
    state.last_origin =origin;
       state.last_points = cout;

end

%
%
cb(1,:) = origin;
count = 2;
for ii=1:4
   if(ii ==  mi)
       continue;
   end
      
   cb(count,:) = cout(ii,:);
   count = count + 1;
end

