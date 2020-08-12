function W=EMD_Gaussian(mean1, cov1, mean2, cov2)
    W=sum((mean1-mean2).^2)+trace(cov1+cov2-2*sqrtm(sqrtm(cov2)*cov1*sqrtm(cov2)));
    W=sqrt(W);
end