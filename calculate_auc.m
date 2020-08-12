% load exp3_0.1noise.mat;
auclist=[];
for jjj=1:1:5
    l=length(roc(:,1,jjj));
    auc=sum((roc(2:l,1,jjj)-roc(1:(l-1),1,jjj)).*roc(2:l,2,jjj));
    auc=abs(auc);
    auclist=[auclist,auc];
end
figure(1);
n=1:1:5;
plot(n,auclist,'r');
hold on;
load exp3_0.05noise.mat;
auclist=[];
for jjj=1:1:10
    l=length(roc(:,1,jjj));
    auc=sum((roc(2:l,1,jjj)-roc(1:(l-1),1,jjj)).*roc(2:l,2,jjj));
    auc=abs(auc);
    auclist=[auclist,auc];
end
% figure(2);
n=1:1:10;
plot(n,auclist,'b');
hold on;

load exp3_0.025noise.mat;
auclist=[];
for jjj=1:1:10
    l=length(roc(:,1,jjj));
    auc=sum((roc(2:l,1,jjj)-roc(1:(l-1),1,jjj)).*roc(2:l,2,jjj));
    auc=abs(auc);
    auclist=[auclist,auc];
end
% figure(2);
n=1:1:10;
plot(n,auclist,'k');
hold on;
load exp3_0.05noise.mat;
auclist=[];
for jjj=1:1:10
    l=length(roc(:,1,jjj));
    auc=sum((roc(2:l,1,jjj)-roc(1:(l-1),1,jjj)).*roc(2:l,2,jjj));
    auc=abs(auc);
    auclist=[auclist,auc];
end
% figure(2);
n=1:1:10;
plot(n,auclist,'g');
thetaz=1;
thetax=1;
