function W=W_EMD_Gaussian(mean1, cov1, mean2, cov2,cp1,cp2)
    W=(1+abs(cp1-cp2)/(cp1+cp2))*sum((mean1-mean2).^2)+(1-abs(cp1-cp2)/(cp1+cp2))*trace(cov1+cov2-2*sqrtm(sqrtm(cov2)*cov1*sqrtm(cov2)));%范围也大概能控制住
    W=sqrt(W);
end