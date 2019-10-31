function out = loc_adaptive_thresh(img, n, m, vt)

out = logical(zeros(size(img)));
for ii = 1:m:size(img,1)
    for jj = 1:n:size(img,2)-1
     
        
        v = var(reshape(double(img(ii:ii+m-1,jj:jj+n-1)),1,[]));
        if(v > vt)
            TR = graythresh(img(ii:ii+m-1,jj:jj+n-1));
            out(ii:ii+m-1,jj:jj+n-1) = im2bw(img(ii:ii+m-1,jj:jj+n-1),TR);
        else
            out(ii:ii+m-1,jj:jj+n-1) = true;
        end
       
       imshow(out);
       drawnow;
    end
end


