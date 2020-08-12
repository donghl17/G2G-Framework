filename='icpList.csv';
pose = importdata(filename);
error=[];
for k=1:1:35
R=[pose.data(k+1,1:3);pose.data(k+1,5:7);pose.data(k+1,9:11)];
T=[pose.data(k+1,4);pose.data(k+1,8);pose.data(k+1,12)];
thetax=atan(R(3,2)/R(3,3));
thetay=atan(-1*R(3,1)/sqrt(R(3,2).^2+R(3,3).^2));
thetaz=atan(R(2,1)/R(1,1));
RR=zeros(3,3);
RR(1,1)=cos(thetay)*cos(thetaz);
RR(1,2)=sin(thetax)*sin(thetay)*cos(thetaz)-cos(thetax)*sin(thetaz);
RR(1,3)=cos(thetax)*sin(thetay)*cos(thetaz)+sin(thetax)*sin(thetaz);
RR(2,1)=cos(thetay)*sin(thetaz);
RR(2,2)=sin(thetax)*sin(thetay)*sin(thetaz)+cos(thetax)*cos(thetaz);
RR(2,3)=cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz);
RR(3,1)=-sin(thetay);
RR(3,2)=sin(thetax)*cos(thetay);
RR(3,3)=cos(thetax)*cos(thetay);
error=[error,sum(R-RR)];
end


%mengtecalo sampling
num=10000;
pointnum=round(CP.*num);
result=zeros(num,3);
occupancy=zeros(num,1);%gorund truth
pro=zeros(num,1);
ptr=1;
for p=1:1:length(CP)%sampling from every component
    mu = MU(p,:);
    sigma = SIGMA(:,:,p);
    R = chol(sigma);
    temp = repmat(mu,pointnum(1,p),1) + randn(pointnum(1,p),3)*R;
    [L,useless]=size(temp);%Length is the number of the array
    if L==0
        continue;
    else
            totalpdf=zeros(L,1);
    for pp=1:1:length(CP)%calculate pro.
        Npdf=mvnpdf(temp,MU(pp,:),SIGMA(:,:,pp));
        totalpdf=totalpdf+Npdf*CP(1,pp);     
    end
    pro(ptr:ptr+L-1,1)=totalpdf;
    result(ptr:ptr+L-1,:)=temp;
    
    for pp=1:1:L
        tempdata=data_total-temp(pp,:);
        distance=sqrt(min(tempdata(:,1).^2+tempdata(:,2).^2+tempdata(:,3).^2));
        if distance<0.1
            occupancy(ptr,1)=1;
            ptr=ptr+1;
%             occupancy=[occupancy;1];
        else
            occupancy(ptr,1)=0;
            ptr=ptr+1;
%             occupancy=[occupancy;0];
        end
        
    end
    end

    
end
final=[result,pro,occupancy];%sampling result+probability+occupancy status
final=sortrows(final,4);
pcount=numel(find(occupancy==1)) ;
fcount=length(occupancy)-pcount;
roc=zeros(length(final),2);
px=0;%p_predict/f_real
py=0;%p_predict/p_real
for p=length(final):-1:1
    if final(p,5)==1
        py=py+1;
    else
        px=px+1;
    end
    roc(p,1)=px/fcount;
    roc(p,2)=py/pcount; 
end
plot(roc(:,1),roc(:,2));

% Plot based on height
% Scatter3 is too slow so we split manually to plot colors
cmap = colormap;
d = length(cmap);
zLow = min(result(:,3));
zDelta = (max(result(:,3))-min(result(:,3)))/d;
figure(4);
hold on
for ppp = 1:length(cmap)
    filter = (result(:,3) > zLow & result(:,3) <= zLow+zDelta);
    plot3(result(filter,1), result(filter,2), result(filter,3), '.', 'Color', cmap(ppp,:))
    zLow = zLow+zDelta;
end
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
grid on
hold off






num=2000;






% x=-5:0.2:14.9;
% y=0:0.1:59.9;
% z=-5:0.2:14.9;
% % [ useless ,a]=size(x);
% % [ useless ,b]=size(y);
% % [ useless ,c]=size(z);
% [xx,yy,zz] = meshgrid(x,y,z);
% temp=[xx(:) yy(:) zz(:)];
% [s,useless]=size(temp);
% totalpdf=zeros(s,1);
% for pp=1:1:i*k
%     Npdf=mvnpdf(temp,MU(pp,:),SIGMA(:,:,pp));
%     totalpdf=totalpdf+Npdf;  
% end  
% result=zeros(1,3);
%  for a=1:1:s       
%                if totalpdf(a,1)>0.005
%                    result=[result;temp(a,:)];
%         
% %                     figure(4)                   
% %                      scatter3(temp(a,1),temp(a,2),temp(a,3), 'b','filled');
% %                     hold on
%                 end
%  end

%test point
% testpoint=[0,0,0;10,0,0;0,10,0;0,0,10;10,10,0;10,0,10;0,10,10;10,10,10;5,5,5;
%            0,20,0;10,20,0;0,40,0;0,20,10;10,40,0;10,20,10;0,40,10;10,40,10;5,25,5;5,30,5;5,35,5;
%            0,50,0;10,50,0;0,60,0;0,50,10;10,60,0;10,50,10;0,60,10;10,60,10;5,55,5];
% % x=testpoint(:,1)';
% % y=testpoint(:,2)'; 
% % z=testpoint(:,3)'; 
% % [xx,yy,zz] = meshgrid(x,y,z);
% % temp=[xx(:) yy(:) zz(:)];
% [s,useless]=size(testpoint);
% totalpdf=zeros(s,1);
% [hang,lie]=size(MU);
% for pp=1:1:hang
%     Npdf=mvnpdf(testpoint,MU(pp,:),SIGMA(:,:,pp));
%     totalpdf=totalpdf+Npdf*CP(1,pp);  
% end  
% figure;
% bar(log(totalpdf)+15);
% 
% 
% aaaaa=3;