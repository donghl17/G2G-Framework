close all,clear all,clc;
%filepath=['C:\Users\f3000\Desktop\2019.9map_compression\Map-compression(1)\Map-compression\GMM'];
i=200;
c=0;
MU=[];
SIGMA=[];
L=[];
CP=[];
storage=[];
runtime=[];
subSampleRatio=0;
%% profiling intialization
% count=0;
% readtime=[];
% GMMtime=[];
% show_GMM_time=[];
% show_map_time=[];
% EM_time=[];
% transform_time=[];
roc=zeros(10000,2,17);
%%  ground truth data
% data_total=[];
% for k=0:1:20
%     filename=['PointCloud',int2str(k),'.csv'];
%     cloud = importdata(filename);
%     datat=cloud.data(:,2:4);
% %     S=find(datat(:,3)<2);
% %     datat=datat(S,:);
%     data_total=[data_total;datat];
% end
load('data_total.mat');

for k=0:1:30%1:5:61  
    %% loading data and initialization
%t1=clock;
%��������
% data = textread(...
%         [filepath '\' num2str(k) '_pointcloud.txt']);    
%     pose = textread(...
%         [filepath '\' num2str(k) '_T.txt']);  
%     R=pose(1:3,1:3);
%     T=pose(1:3,4);
% if k==1
%filename=['/home/yujc/GMMmap/ETH_global/global_frame/PointCloud',int2str(1),'.csv'];
% else
    filename=['PointCloud',int2str(k),'.csv'];
% end
% 
% filename=['Hokuyo_',int2str(k),'.csv'];
cloud = importdata(filename);
data=cloud.data(:,2:4);
filename=['PointCloud',int2str(30),'.csv'];
cloud = importdata(filename);
data1=cloud.data(:,2:4);
data=[data;data1];
% data=data_total(:,:);
R=[1,0,0;0,1,0;0,0,1];
T=[0;0;0];
% f = rand(1, length(data));
% data=data(f>subSampleRatio,:);
% filename=['RT_',int2str(k),'.mat'];
% load(filename);
% A=zeros(4,4);
% A(4,4)=1;
% A(1:3,1:3)=R';
% A(4,1:3)=T';
% A=affine3d(A);
% pt = pointCloud(data);
% data= pctransform(pt,A);% input and output are all "pointcloud" class 
% data=data.Location;
% filename=['target2_frame',int2str(k),'_xyz.mat'];
% load(filename);
% choose=(data(:,1)>xt).*(data(:,1)<(xt+6)).*(data(:,2)>yt).*(data(:,2)<(yt+6)).*(data(:,3)>zt).*(data(:,3)<(zt+6));
% data=data(choose==1,:);
% x=data(:,1);
% y=data(:,2);
% z=data(:,3);
% A=invert(A);
% pt = pointCloud([x,y,z]);
% data_target= pctransform(pt,A);
% load target2.mat;
% pt1=pointCloud([x,y,z]);
% tform = pcregisterndt(pt,pt1,0.5 );
% tform = invert(tform);
% RT=tform.T;
% save( ['regis_semantic_',int2str(k),'.mat'],'RT');
% filename='icpList.csv';
% pose = importdata(filename);
% R=[pose.data(k+1,1:3);pose.data(k+1,5:7);pose.data(k+1,9:11)];
% T=[pose.data(k+1,4);pose.data(k+1,8);pose.data(k+1,12)];
%     load( ['regis_',int2str(k),'mat']);
%     R=RT(1:3,1:3)';
%     T=RT(4,1:3)';


%% adding pose noise
% noise=0.075;
% R1_error=unifrnd(-1,1)*noise;
% R2_error=unifrnd(-1,1)*noise;
% R3_error=unifrnd(-1,1)*noise;
% T1_error=unifrnd(-1,1)*noise;
% T2_error=unifrnd(-1,1)*noise;
% T3_error=unifrnd(-1,1)*noise;
% thetax=atan(R(3,2)/R(3,3))*(1+R1_error);
% thetay=atan(-1*R(3,1)/sqrt(R(3,2).^2+R(3,3).^2))*(1+R2_error);
% thetaz=atan(R(2,1)/R(1,1))*(1+R2_error);
% RR=zeros(3,3);
% RR(1,1)=cos(thetay)*cos(thetaz);
% RR(1,2)=sin(thetax)*sin(thetay)*cos(thetaz)-cos(thetax)*sin(thetaz);
% RR(1,3)=cos(thetax)*sin(thetay)*cos(thetaz)+sin(thetax)*sin(thetaz);
% RR(2,1)=cos(thetay)*sin(thetaz);
% RR(2,2)=sin(thetax)*sin(thetay)*sin(thetaz)+cos(thetax)*cos(thetaz);
% RR(2,3)=cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz);
% RR(3,1)=-sin(thetay);
% RR(3,2)=sin(thetax)*cos(thetay);
% RR(3,3)=cos(thetax)*cos(thetay);
% R=RR;
% T(1,1)=T(1,1)*(1+T1_error);
% T(2,1)=T(2,1)*(1+T1_error);
% T(3,1)=T(3,1)*(1+T1_error);

% t2=clock;  
% readtime=[readtime,etime(t2,t1)];
options = statset('MaxIter',1000);
GMModel1 = fitgmdist(data,i,'RegularizationValue',0.1,'Options',options);%'RegularizationValue',0.0001
 fprintf([int2str(k),' round construction']);
% t_temp=clock;
% EM_time=[EM_time,etime(t_temp,t2)];
mu1=GMModel1.mu';
mu1=R*mu1;
for p=1:i
    mu1(:,p)=mu1(:,p)+T;
end
mu1=mu1';
Sigma1=GMModel1.Sigma;
for i1=1:i
Sigma1(:,:,i1)=R*Sigma1(:,:,i1)*R';
end
% t3=clock;
% transform_time=[transform_time,etime(t3,t_temp)];
% GMMtime=[GMMtime,etime(t3,t2)];


%% semantics detection
% xmin=min(mu1(:,1));
% ymin=min(mu1(:,2));
% zmin=min(mu1(:,3));
% xmax=max(mu1(:,1));
% ymax=max(mu1(:,2));
% zmax=max(mu1(:,3));
%window x=5 y=5 z=5
% load target2.mat;
% difference_avg=10000;
% for p=0:0.5:4%-2:0.5:2
%     for q=8:0.5:12%-4:0.5:0
%         for r=7:0.5:11%-2:0.5:2
%             %choose=(mu1(:,1)>p).*(mu1(:,1)<(p+5)).*(mu1(:,2)>q).*(mu1(:,2)<(q+5)).*(mu1(:,3)>r).*(mu1(:,3)<(r+5));
%             choose=(mu1(:,1)>p).*(mu1(:,1)<(p+6)).*(mu1(:,2)>q).*(mu1(:,2)<(q+6)).*(mu1(:,3)>r).*(mu1(:,3)<(r+6));
%             component_num=sum(choose);
%             if component_num>7 % if target , component_num is (30,40),about 40average
%             choose_no=find(choose==1);
%             mu_temp=mu1(choose_no,:);
%             Sigma_temp=Sigma1(:,:,choose_no);
%             compare= fitgmdist([x,y,z],component_num,'RegularizationValue',0.1,'Options',options);%'RegularizationValue',0.0001
%             fprintf(['find one:',int2str(component_num),'/n']);
%             kl_list=zeros(component_num,component_num);
%             for pp=1:1:component_num% target components
%                 for qq=1:1:component_num%map components
%                     kl_temp = EMD_Gaussian(mu_temp(pp,:),Sigma_temp(:,:,pp),compare.mu(qq,:),compare.Sigma(:,:,qq));%EM距离
%                     kl_list(pp,qq)=kl_temp;
%                 end
%             end
%             if (sum(min(kl_list))/component_num)<difference_avg
%             difference_avg=sum(min(kl_list))/component_num;
%             target_find=choose_no;
%             xt=p;
%             yt=q;
%             zt=r;
%             end
%             end
%         end
%     end
% end
% cp=GMModel1.ComponentProportion(:,target_find);
% mu=mu1(target_find,:);
% sigma=Sigma1(:,:,target_find);
% save( ['target2_frame',int2str(k),'_xyz.mat'], 'xt', 'yt', 'zt','cp','mu','sigma');
% fprintf([int2str(k),' round detection']);
% 


%% G2G updating and merging
% if k==1%need to adjust according to intial status
%     %direct add first frame into global map
%     MU=[MU;mu1];
%     SIGMA=cat(3,SIGMA,Sigma1);
%     CP=[CP,GMModel1.ComponentProportion];
%     storage=[storage,i];
% else 
%     %calculate EM distance and do novelty check
%     size1=size(SIGMA);
%     sizek=size1(1,3);%�ɳɷ���
%     zh=zeros(4,3,sizek);
%     for v=1:sizek
%         zh(1:3,1:3,v)=SIGMA(:,:,v);
%         zh(4,1:3,v)=MU(v,1:3);
%     end
%     size1=size(Sigma1);
%     sizenew=size1(1,3);%�³ɷ���
%     NO_old=(1:sizek);
%     NO_new=(1:sizenew);
%     [x,y]=meshgrid(NO_new,NO_old);
%     [hang,lie]=size(x);
%     count=0;
%     merge_new=[];
%     merge_old=[];
%     W_EM_distance=[];
%     for g=1:lie
%         [hang,lie]=size(x);
%         L=zeros(hang,1);%������hang��sizekӦ�����
%         for gg=1:hang
% %             kl = kldivGaussian(zh(4,1:3,y(gg,g)), zh(1:3,1:3,y(gg,g)),mu1(x(gg,g),:),Sigma1(:,:,x(gg,g)));
%               kl = EMD_Gaussian(zh(4,1:3,y(gg,g)), zh(1:3,1:3,y(gg,g)),mu1(x(gg,g),:),Sigma1(:,:,x(gg,g)));%EM距离
%             % kl = W_EMD_Gaussian(zh(4,1:3,y(gg,g)), zh(1:3,1:3,y(gg,g)),mu1(x(gg,g),:),Sigma1(:,:,x(gg,g)),CP(1,y(gg,g)),GMModel1.ComponentProportion(1,x(gg,g)));%EM距离
%             L(gg,1)=kl;
%         end
%         W_EM_distance=[W_EM_distance,min(L)];
%         if min(L)<3
%             [hang,lie]=find(L==min(L));%�к�Ϊ1����ָ�Ѵ��ڳɷ��еı��
%             merge_new=[merge_new,g];
%             merge_old=[merge_old,hang];
%         else
%             count=count+1;%�³ɷּ���
%         end
%     end
%     if count<floor(i/20)
%         ֡%% too less novel information, discard this frame of map
%     else
%         for g=1:sizenew
%             if find(merge_new==g)
%                 no=find(merge_new==g);
%                 no=merge_old(no);
%                 MU(no,:)=0.5*(MU(no,:)+GMModel1.mu(g,:));
%                 SIGMA(:,:,no)=0.25*(SIGMA(:,:,no)+GMModel1.Sigma(:,:,g));
%                 CP(no)=0.5*(CP(no)+GMModel1.ComponentProportion(1,g));
%             else
%                 MU=[MU;mu1(g,:)];
%                 SIGMA=cat(3,SIGMA,Sigma1(:,:,g));
%                 CP=[CP,GMModel1.ComponentProportion(1,g)];
%             end
%             
%         end
% 
%     end
%     storage=[storage,count];
% end

%%clustering (wasted)
% labelno=8;
% labelcount=i;
% list=[];%fenzuqingkuang
% kl_list=zeros(i,i);
% for n=1:1:i%create table
% list=[list;n];
% for m=n:1:i
%     if m==n
%         kl_list(n,m)=10000;
%     else
%     kl_in = EMD_Gaussian(mu1(n,:),Sigma1(:,:,n),mu1(m,:),Sigma1(:,:,m));%EM距离
%     kl_list(n,m)=kl_in;
%     kl_list(m,n)=kl_in;
% end
% end
% end
% while labelcount>labelno
%     [x,y]=find(kl_list==min(min(kl_list)));
%     [hang,lie]=size(list);
%     temp=zeros(i,lie);
%     temp(y(1),:)=list(x(1),:);
%     list=[list,temp];
%     list(x(1),1)=0;
%     list(:,find(sum(abs(list),1)==0))=[];
%     for n=1:1:i
%         kl_list(y(1),n)=(kl_list(y(1),n)+kl_list(x(1),n))/2;
%         kl_list(y(1),n)=(kl_list(n,y(1))+kl_list(n,x(1)))/2;
%         kl_list(x(1),n)=10000;
%         kl_list(n,x(1))=10000;
%         kl_list(y(1),y(1))=10000;
%     end
%     labelcount=labelcount-1;
% end        
% labellist=find(list(:,1)~=0);
% summary=list(labellist,:);
% result=zeros(1,i);
% for n=1:1:labelno
%     temp=summary(n,:);
%     temp(temp==0)=[];
%     result(1,temp)=n;
% end
% 
% tt2=clock;
% runtime=[runtime,etime(tt2,t2)];





    %% directlly merging
    MU=[MU;mu1];
    SIGMA=cat(3,SIGMA,Sigma1);
    CP=[CP,GMModel1.ComponentProportion];



%% visualization 
for n=1:i
Mu =mu1(n,1:3);
Sigma=Sigma1(1:3,1:3,n);
[V,D]=eig(Sigma);
r =1.7;
xhalf = linspace(sqrt(r^2*D(1,1)),0,10);
Ninthalf = round(10/2);
zsect = zeros(10,Ninthalf);
ysect = zeros(10,Ninthalf);
for ti = 1:10
r2d = r^2 - xhalf(ti).^2/D(1,1);
ysect(ti,:) = linspace(0,sqrt(r2d*D(2,2)),Ninthalf);
zsect(ti,:) = sqrt((r2d - ysect(ti,:).^2/D(2,2) )*D(3,3));
xsect(ti,1:Ninthalf) = xhalf(ti);
end
zsect = real(zsect);
%x&gt;0,Z&gt;0
xsect = [xsect,xsect];
ysect = [ysect,fliplr(ysect)];
zsect = [zsect,-fliplr(zsect)];
%x&gt;0
xsect = [xsect,xsect];
ysect = [ysect,-fliplr(ysect)];
zsect = [zsect,fliplr(zsect)];
% make it a whole
xsect = [xsect;-flipdim(xsect,1)];
ysect = [ysect;flipdim(ysect,1)];
zsect = [zsect;flipdim(zsect,1)];
% rotate
[lr,lc] = size(xsect);
for ti = 1:lr
for tj = 1:lc
newcodi = [xsect(ti,tj),ysect(ti,tj),zsect(ti,tj)]*inv(V);
xsect(ti,tj) = newcodi(1);
ysect(ti,tj) = newcodi(2);
zsect(ti,tj) = newcodi(3);
end
end
% shift
xsect = xsect+Mu(1);
ysect = ysect+Mu(2);
zsect = zsect+Mu(3);
xsect1=real(xsect);
ysect1=real(ysect);
zsect1=real(zsect);
% if sum(target_find==n)>0
      figure(3);
%      cdata=cat(3,ones(size(xsect1)),zeros(size(ysect1)),zeros(size(zsect1)));%红色回
% else   
%     figure(4);
%     cdata=cat(3,zeros(size(xsect1)),ones(size(ysect1)),zeros(size(zsect1)));%绿色答
% end

%%color choosing
% switch result(1,n)
%     case 1
% %         mymap=[0 0 0];
%          cdata=cat(3,ones(size(xsect1)),zeros(size(ysect1)),zeros(size(zsect1)));%红色回
%     case 2
% %         mymap=[0 0 1];
%  cdata=cat(3,zeros(size(xsect1)),ones(size(ysect1)),zeros(size(zsect1)));%绿色答
%     case 3
% %         mymap=[0 1 0];
%  cdata=cat(3,zeros(size(xsect1)),zeros(size(ysect1)),ones(size(zsect1)));%兰色
% case 4
% %     mymap=[1 0 0];
% cdata=cat(3,zeros(size(xsect1)),zeros(size(ysect1)),zeros(size(zsect1)));
%     case 5
% %         mymap=[0 1 1];
% cdata=cat(3,zeros(size(xsect1)),ones(size(ysect1)),ones(size(zsect1)));
%         case 6
% %             mymap=[1 0 1];
% cdata=cat(3,ones(size(xsect1)),zeros(size(ysect1)),ones(size(zsect1)));
%     case 7
% %         mymap=[1 1 0];
% cdata=cat(3,ones(size(xsect1)),ones(size(ysect1)),zeros(size(zsect1)));
%     otherwise
% %         mymap=[1 1 1];
% cdata=cat(3,ones(size(xsect1)),ones(size(ysect1)),ones(size(zsect1)));
% end

h3=surf(xsect1,ysect1,zsect1);

alpha(0.3)
xsect=[];
xsect1=[];
ysect=[];
ysect1=[];
zsect=[];
zsect1=[];
title('{\bf Scatter Plot and Fitted Gaussian Mixture Contours}')
hold on;
end
figure(3);
grid off;
shading interp;


% t4=clock;
% show_GMM_time=[show_GMM_time,etime(t4,t3)];
% t5=clock;
% show_map_time=[show_map_time,etime(t5,t4)];

%% mengtecalo sampling and calculating roc
% fprintf(['calculating roc',int2str(k)]);
% num=10000;
% pointnum=round(CP.*num./sum(CP));
% result=zeros(num,3);
% occupancy=zeros(num,1);%gorund truth
% pro=zeros(num,1);
% ptr=1;
% for p=1:1:length(CP)%sampling from every component
%     mu = MU(p,:);
%     sigma = SIGMA(:,:,p);
%     R = chol(sigma);
%     temp = repmat(mu,pointnum(1,p),1) + randn(pointnum(1,p),3)*R;
%     [L,useless]=size(temp);%Length is the number of the array
%     if L==0
%         continue;
%     else
%             totalpdf=zeros(L,1);
%     for pp=1:1:length(CP)%calculate pro.
%         Npdf=mvnpdf(temp,MU(pp,:),SIGMA(:,:,pp));
%         totalpdf=totalpdf+Npdf*CP(1,pp);     
%     end
%     pro(ptr:ptr+L-1,1)=totalpdf;
%     result(ptr:ptr+L-1,:)=temp;
%     
%     for pp=1:1:L
%         tempdata=data_total-temp(pp,:);
%         distance=sqrt(min(tempdata(:,1).^2+tempdata(:,2).^2+tempdata(:,3).^2));
%         if distance<0.5
%             occupancy(ptr,1)=1;
%             ptr=ptr+1;
% %             occupancy=[occupancy;1];
%         else
%             occupancy(ptr,1)=0;
%             ptr=ptr+1;
% %             occupancy=[occupancy;0];
%         end
%     end
%     end
%    
% end
% final=[result,pro,occupancy];%sampling result+probability+occupancy status
% final=sortrows(final,4);
% pcount=numel(find(occupancy==1)) ;
% fcount=length(occupancy)-pcount;
% % roc=zeros(length(final),2);
% px=0;%p_predict/f_real
% py=0;%p_predict/p_real
% for p=length(final):-1:1
%     if final(p,5)==1
%         py=py+1;
%     else
%         px=px+1;
%     end
%     roc(p,1,(k+1)/2)=px/fcount;
%     roc(p,2,(k+1)/2)=py/pcount; 
% end


%    fprintf(['finish round',int2str(k)]); 
end
save exp_semantic_0.1noise_200com.mat roc storage runtime CP MU SIGMA;
 saveas(gca,'segment.fig');

%% sampling and visualization
% num=8000;
% pointnum=round(CP.*num/sum(CP));
% result=[];
% for p=1:1:length(CP)
%     mu = MU(p,:);
%     sigma = SIGMA(:,:,p);
%     R = chol(sigma);
%     temp = repmat(mu,pointnum(1,p),1) + randn(pointnum(1,p),3)*R;
%     result=[result;temp];
% end
% 
% %%
% % figure;
% % plot(roc(:,1,(k+1)/2),roc(:,2,(k+1)/2));
% 
% % Plot based on height
% % Scatter3 is too slow so we split manually to plot colors
% cmap = colormap;
% d = length(cmap);
% zLow = min(result(:,3));
% zDelta = (max(result(:,3))-min(result(:,3)))/d;
% figure(4);
% hold on
% for ppp = 1:length(cmap)
%     filter = (result(:,3) > zLow & result(:,3) <= zLow+zDelta);
%     plot3(result(filter,1), result(filter,2), result(filter,3), '.', 'Color', cmap(ppp,:))
%     zLow = zLow+zDelta;
% end
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% axis equal
% grid on
% hold off
%  saveas(gca,'1_20method.fig');
%  aaaa=1;
% 

