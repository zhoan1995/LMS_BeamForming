%This function performs the LMS algorithm 
function [y, w] = LMSBF(Geometry, pars, doas, chOut, waveform,n_iter)
    %Computing the number of elements of the array antenna
    array_row = size(chOut,1);
    array_col = size(chOut,2);
    %Computing the steering vector
    steervec = phased.SteeringVector('SensorArray', Geometry.BSarray);
    S = steervec(pars.fc, doas);
    P_s = rms(waveform(:,1))^2;%Computing the power of the signal
    P_n = P_s / db2pow(pars.SNR);%Computing power of the noise
    U = P_s * eye(size(doas,2));
    R_u = S*U*S' + eye(array_row) * P_n; %each diagonal element of Ru=avg power measured on the corresponding elements of array
    
    
    W_ = ones(array_row, array_col*n_iter);
    mu = 1/trace(R_u); %Initial mu, step size, increas mu__>increas miss adjusment noise
    %mu = 0.5;
    W_outIter = ones(n_iter,array_row);
    coun = 1;
    for j = 1:n_iter
        for i = 1:array_col
            coun = coun + 1;
            u = chOut(:,i); %Received signal
            d = waveform(i); %Desired signal
            if i==1
              e = (W_(:,1)'*u) - d; %Error for first time
            else
               e = (( W_(:,coun-1) - mu*u*conj(e))'*u) - d; %Error
            end
            W_(:,coun) =W_(:,coun-1) - mu*u*conj(e);%LMS formula from the slides
        end
        mu = mu * 0.9; %Reducing the mu in each iteration 
        W_outIter(j,:) = transpose(W_(:,coun-1) - mu*u*conj(e));
    end
    %Computing the array output
    y = transpose((W_(:,coun-1) - mu*u*conj(e))' * chOut);
    w = W_(:,coun-1) - mu*u*conj(e);
end