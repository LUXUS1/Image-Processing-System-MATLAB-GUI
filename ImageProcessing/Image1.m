function varargout = Image1(varargin)
% IMAGE1 MATLAB code for Image1.fig
%      IMAGE1, by itself, creates a new IMAGE1 or raises the existing
%      singleton*.
%
%      H = IMAGE1 returns the handle to a new IMAGE1 or the handle to
%      the existing singleton*.
%
%      IMAGE1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMAGE1.M with the given input arguments.
%
%      IMAGE1('Property','Value',...) creates a new IMAGE1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Image1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Image1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Image1

% Last Modified by GUIDE v2.5 24-May-2020 13:49:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Image1_OpeningFcn, ...
                   'gui_OutputFcn',  @Image1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Image1 is made visible.
function Image1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Image1 (see VARARGIN)

% Choose default command line output for Image1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Image1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Image1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'XColor',get(gca,'Color')) ;% 这两行代码功能：将坐标轴和坐标刻度转为白色
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % 这两行代码功能：去除坐标刻度
set(gca,'YTickLabel',[]);
% Hint: place code in OpeningFcn to populate axes1


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'XColor',get(gca,'Color')) ;% 这两行代码功能：将坐标轴和坐标刻度转为白色
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % 这两行代码功能：去除坐标刻度
set(gca,'YTickLabel',[]);
% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'XColor',get(gca,'Color')) ;% 这两行代码功能：将坐标轴和坐标刻度转为白色
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % 这两行代码功能：去除坐标刻度
set(gca,'YTickLabel',[]);
% Hint: place code in OpeningFcn to populate axes3


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    cla(handles.axes1);
    cla(handles.axes2);
    cla(handles.axes3);
    global im %定义一个全局变量  
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'载入图像');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('没有选中文件','出错');
    return;
    else 
    str=[pathname,filename];
    end
    %%打开图像  
    im=imread(str);  
    %%打开axes1的句柄 进行axes1的操作  
    axes(handles.axes1);  
    %%在axes1中显示原始图像  
    imshow(im);  
    


% --- Executes on button press in pushbutton2.
%% 基本处理
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu1, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('请选择实现方式','出错');
            end
        case 2
            im1= rgb2gray(im);%灰度处理
            im2=imnoise(im1,'salt & pepper',0.04);%添加椒盐噪声
            %%打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            %%在axes2中显示灰度图像
            imshow(im2),xlabel('噪声图像','Fontsize',12);
            im3 = medfilt2(im2,[2,4]);%去噪处理
            %%打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            %%在axes2中显示去噪后图像
            imshow(im3);
        case 3
            im1= rgb2gray(im);%灰度处理
            %%打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            %%在axes2中显示灰度图像
            imshow(im1);
            im2 = histeq(im1);
            %%打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            %%在axes2中显示直方图像
            imhist(im2);
    end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
%% 边缘检测
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu2, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('请选择实现方式','出错');
            end
        case 2
            im1= rgb2gray(im);
            %打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            %在axes2中显示灰度图像
            imshow(im1);
            %使用canny算子进行边缘处理
            im1=double(im1);%将图像转化为double型
            thresh=[0.02,0.2];%通过遍历灰度图中点，将图像信息二值化，处理过后的图片只有二种色值。
            sigma=2;%指定sigma，即滤波器的标准偏差
            im2=edge(im1,'canny',thresh,sigma);
            %%打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            %%在axes3中显示边缘图像
            imshow(im2,[]);
        case 3
            im1= rgb2gray(im);
            %使用canny算子进行边缘处理
            im1=double(im1);
            thresh=[0.02,0.2];
            sigma=2;
            im2=edge(im1,'canny',thresh,sigma);
            %%打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            %%在axes2中显示边缘图像
            imshow(im2,[]);
            [H,theta,rho]=hough(im2,'RhoResolution',0.5,'ThetaResolution',0.5);
            peak=houghpeaks(H,5);%求极值点
            lines=houghlines(im2,theta,rho,peak);%返回原图直线信息
            %打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            %在axes3中显示灰度图像
            imshow(im2),hold on
            for k=1:length(lines)
                xy=[lines(k).point1;lines(k).point2];
                max_len=0;
                len=norm(lines(k).point1-lines(k).point2);
                if(len>max_len)
                    xy_long=xy;
                    %绘制线的起点和终点
                    plot(xy(1,1),xy(1,2),'xw','LineWidth',2,'Color','y');%起点
                    plot(xy(2,1),xy(2,2),'xw','LineWidth',2,'Color','r');%终点
                    plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','c');%画出线段
                end
            end
    end

% --- Executes on button press in pushbutton5.
%% 图像分割
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu4, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('请选择实现方式','出错');
            end
        case 2
            im=rgb2gray(im);%转换为灰度图
            %打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            imshow(im);
            I_double=double(im);
            [wid,len]=size(im);%图像的大小
            %灰度级
            colorLevel=256;
            %直方图
            hist=zeros(colorLevel,1);
            %计算直方图
            for i=1:wid
                for j=1:len
                    m=im(i,j)+1;%图像的灰度级m
                    hist(m)=hist(m)+1;%灰度值为i的像素和
                end
            end
            %直方图归一化
            hist=hist/(wid*len);%各灰度值概率 Pi
            miuT=0;%定义总体均值
            for m=1:colorLevel
                miuT=miuT+(m-1)*hist(m);  %总体均值
            end
            xigmaB2=0;%
            for mindex=1:colorLevel
                threshold=mindex-1;%设定阈值
                omega1=0;%目标概率
                for m=1:threshold-1
                    omega1=omega1+hist(m);% 目标概率 W0
                end
                omega2=1-omega1; %背景的概率 W1
                miu1=0;%目标的平均灰度值
                miu2=0;%背景的平均灰度值
                for m=1:colorLevel
                    if m<threshold
                        miu1=miu1+(m-1)*hist(m);%目标 i*pi的累加值[1 threshold]
                    else
                        miu2=miu2+(m-1)*hist(m);%背景 i*pi的累加值[threshold m]
                    end
                end
                miu1=miu1/omega1;%目标的平均灰度值
                miu2=miu2/omega2;%背景的平均灰度值
                xigmaB21=omega1*(miu1-miuT)^2+omega2*(miu2-miuT)^2;%最大方差
                xigma(mindex)=xigmaB21;%先设定一个值 再遍历所有灰度级
                %找到xigmaB21的值最大
                if xigmaB21>xigmaB2
                    finalT=threshold;%找到阈值 灰度级
                    xigmaB2=xigmaB21;%方差为最大
                end
            end
            %阈值归一化
            for i=1:wid
                for j=1:len
                    if I_double(i,j)>finalT %大于所设定的均值 则为目标
                        bin(i,j)=0;
                    else
                        bin(i,j)=1;
                    end
                end
            end
            %打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            imshow(bin);
        case 3            
            % 转化为灰度图再求
            gray=rgb2gray(im);
            [m,n]=size(gray);
            k=3; % 聚类个数
            % 将图像进行RGB——3通道分解
            A = reshape(im(:, :, 1), m*n, 1);
            B = reshape(im(:, :, 2), m*n, 1);
            C = reshape(im(:, :, 3), m*n, 1);
            data = [A B C];                         
            % 过渡图像
            res = kmeans(double(data), k);
            result = reshape(res, m, n);             % 反向转化为图片形式
            %打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            % label2rgb功能是转换标记矩阵到RGB图像
            imshow(label2rgb(result)),
            title(strcat('K=',num2str(k),'时RGB通道分割结果'));   % 显示分割结果           
            % 处理结果
            res = kmeans(double(data), k+2);
            result = reshape(res, m, n);
            %打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            imshow(label2rgb(result)),
            title(strcat('K=',num2str(k+2),'时RGB通道分割结果')); % 显示分割结果

    end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
%% 选择第一张图
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    cla(handles.axes1);
    cla(handles.axes2);
    cla(handles.axes3);
    global ima1
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'载入图像');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('没有选中文件','出错');
    return;
    else 
    str=[pathname,filename];
    end
    %%打开图像  
    ima1=imread(str);  
    %%打开axes1的句柄 进行axes1的操作  
    axes(handles.axes1);  
    %%在axes1中显示原始图像  
    imshow(ima1),hold on  

% --- Executes on button press in pushbutton8.
%% 选择第二张图
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global ima2
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'载入图像');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('没有选中文件','出错');
    return;
    else 
    str=[pathname,filename];
    end
    %%打开图像  
    ima2=imread(str);
    %%打开axes2的句柄 进行axes2的操作  
    axes(handles.axes2);  
    %%在axes2中显示原始图像  
    imshow(ima2);


% --- Executes on button press in pushbutton9.
%% 目标检测
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
      global im
        a=get(handles.popupmenu8, 'value');
        switch a
            case 1
                if real(a)==1
                    errordlg('请选择实现方式','出错');
                end
            case 2
                I1=rgb2gray(im);%功能是将真彩色图像转换为灰度图像，即灰度化处理
                %%打开axes2的句柄 进行axes2的操作
                axes(handles.axes2);
                %%在axes2中显示原始图像
                imshow(I1);
                I2=edge(I1,'Prewitt',0.15,'both');
                %功能是采用I1作为它的输入，并返回一个与I1相同大小的二值化图像I2，
                %在函数检测到边缘的地方为1，其他地方为0
                se=[1;1;1];
                I3=imerode(I2,se);%腐蚀
                se=strel('rectangle',[25,25]);
                I4=imclose(I3,se);
                I5=bwareaopen(I4,2000);%作用是删除二值图像BW中面积小于2000的对象
                [y,x]=size(I5);
                myI=double(I5);%double类型
                Blue_y=zeros(y,1);%zeros功能是返回一个m×n×p×...的double类零矩阵
                for i=1:y
                    for j=1:x
                        if(myI(i,j,1)==1)
                            
                            Blue_y(i,1)= Blue_y(i,1)+1;%蓝色像素点统计
                        end
                    end
                end
                [~, MaxY]=max(Blue_y);%Y方向车牌区域确定
                PY1=MaxY;
                while ((Blue_y(PY1,1)>=5)&&(PY1>1))
                    PY1=PY1-1;
                end
                PY2=MaxY;
                while ((Blue_y(PY2,1)>=5)&&(PY2<y))
                    PY2=PY2+1;
                end
                %%%%%% X方向 %%%%%%%%%
                Blue_x=zeros(1,x);%进一步确定x方向的车牌区域
                for j=1:x
                    for i=PY1:PY2
                        if(myI(i,j,1)==1)
                            Blue_x(1,j)= Blue_x(1,j)+1;
                        end
                    end
                end
                
                PX1=1;
                while ((Blue_x(1,PX1)<3)&&(PX1<x))
                    PX1=PX1+1;
                end
                PX2=x;
                while ((Blue_x(1,PX2)<3)&&(PX2>PX1))
                    PX2=PX2-1;
                end
                PX1=PX1-1;%对车牌区域的校正
                PX2=PX2+1;
                dw=im(PY1:PY2-8,PX1:PX2,:);
                bh=rgb2gray(dw);%功能是将真彩色图像转换为灰度图像，即灰度化处理
                %%打开axes3的句柄 进行axes3的操作
                axes(handles.axes3);
                %%在axes2中显示原始图像
                imshow(bh);
            case 3
                im = rgb2gray(im);
                % 设置阈值
                bw = imbinarize(im,0.69);%二值化
                % 通过领域判断手动去噪
                [m,n] = size(bw);
                for i = 2:m-1
                    for j = 2:n-1
                        %同上下元素判断
                        if(bw(i,j)~=bw(i+1,j) && bw(i,j)~=bw(i-1,j))
                            bw(i,j) = 1;
                            %同左右元素判断
                        elseif(bw(i,j)~=bw(i,j+1) && bw(i,j)~=bw(i,j-1))
                            bw(i,j) = 1;
                            %同斜边元素判断
                        elseif(bw(i,j)~=bw(i+1,j+1) && bw(i,j)~=bw(i-1,j-1))
                            bw(i,j) = 1;
                            %同斜边元素判断
                        elseif(bw(i,j)~=bw(i-1,j+1) && bw(i,j)~=bw(i+1,j-1))
                            bw(i,j) = 1;
                        end
                    end
                end
                
                % 二值化图像取反
                for i = 1:m
                    for j = 1:n
                        bw(i,j) = ~bw(i,j);
                    end
                end
                %打开axes2的句柄 进行axes2的操作
                axes(handles.axes2);
                imshow(bw);
                bw = bwareaopen(bw,30);%删除二值图像BW中面积小于bw的对象，默认情况下使用8邻域。
                % 图形学结构元素构建，圆形半径为8
                se = strel('disk',8);
                % 关操作
                bw = imclose(bw,se);
                % 填充孔洞
                bw = imfill(bw,'holes');
                [B,L] = bwboundaries(bw,'noholes');
                %打开axes3的句柄 进行axes3的操作
                axes(handles.axes3);
                imshow(label2rgb(L,@jet,[.5 .5 .5])),hold on;
                for k = 1:length(B)
                    boundary = B{k};
                    % 显示白色边界
                    plot(boundary(:,2),boundary(:,1),'w','LineWidth',2)
                end
                % 确定圆形目标
                stats = regionprops(L,'Area','Centroid');
                % 设置求面积
                threshold = 0.85;%设置阈值
                for k = 1:length(B)
                    boundary = B{k};
                    delta_sq = diff(boundary).^2;
                    % 求周长
                    perimeter = sum(sqrt(sum(delta_sq,2)));
                    % 求面积
                    area = stats(k).Area;
                    metric = 4*pi*area/perimeter^2;
                    metric_string = sprintf('%2.2f',metric);
                    % 根据阈值匹配
                    if metric > threshold
                        centroid = stats(k).Centroid;
                        plot(centroid(1),centroid(2),'ko');
                        text(centroid(1)-2,centroid(2)-2, '这是圆形','Color',...
                            'k','FontSize',14,'FontWeight','bold');
                    end
                    text(boundary(1,2)-10,boundary(1,1)-12, metric_string,'Color',...
                        'k','FontSize',14,'FontWeight','bold');
                end
        end
           

% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7


% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton11.
%% PCA人脸识别
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
        cla(handles.axes1);
        cla(handles.axes2);
        cla(handles.axes3);
        global reference
        global W
        global imgmean
        global col_of_data
        global pathname
        global img_path_list

        % 批量读取指定文件夹下的图片128*128
        pathname = uigetdir;
        img_path_list = dir(strcat(pathname,'\*.bmp'));
        img_num = length(img_path_list);
        imagedata = [];
        if img_num >0
            for j = 1:img_num
                img_name = img_path_list(j).name;
                temp = imread(strcat(pathname, '/', img_name));
                temp = double(temp(:));
                imagedata = [imagedata, temp];
            end
        end
        col_of_data = size(imagedata,2);
        % 中心化 & 计算协方差矩阵
        imgmean = mean(imagedata,2);
        for i = 1:col_of_data
            imagedata(:,i) = imagedata(:,i) - imgmean;
        end
        covMat = imagedata'*imagedata;
        [COEFF, latent, explained] = pcacov(covMat);
        % 选择构成95%能量的特征值
        i = 1; proportion = 0;
        while(proportion < 95)
            proportion = proportion + explained(i);
            i = i+1;
        end
        p = i - 1; 
        % 特征脸
        W = imagedata*COEFF;    % N*M阶
        W = W(:,1:p);           % N*p阶
        % 训练样本在新座标基下的表达矩阵 p*M
        reference = W'*imagedata;

% --- Executes on button press in pushbutton12.
%% PCA人脸识别
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ima;
[filename, pathname] = uigetfile({'*.bmp'},'choose photo');
str = [pathname, filename];
ima = imread(str);
axes( handles.axes1);
imshow(ima);

% --- Executes on button press in pushbutton13.
%% PCA人脸识别
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ima
global reference
global W
global imgmean
global col_of_data
global pathname
global img_path_list

% 预处理新数据
ima = double(ima(:));
objectone = W'*(ima - imgmean);
distance = 100000000;

% 最小距离法，寻找和待识别图片最为接近的训练图片
for k = 1:col_of_data
    temp = norm(objectone - reference(:,k));
    if(distance>temp)
        aimone = k;
        distance = temp;
        % 显示测试结果
        aimpath = strcat(pathname, '/', img_path_list(aimone).name);
        axes( handles.axes2 );
        imshow(aimpath);
    end
end

% --- Executes on button press in pushbutton14.
%% PCA人脸识别
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 选择测试集

global W
global reference
col_of_data = 60;pathname = uigetdir;
img_path_list = dir(strcat(pathname,'\*.bmp'));
img_num = length(img_path_list);testdata = [];
if img_num >0
    for j = 1:img_num
        img_name = img_path_list(j).name;
        temp = imread(strcat(pathname, '/', img_name));
        temp = double(temp(:)); testdata = [testdata, temp];
    end
end
col_of_test = size(testdata,2);testdata = center( testdata );
object = W'* testdata;
% 最小距离法，寻找和待识别图片最为接近的训练图片， 计算分类器准确率
num = 0;
for j = 1:col_of_test
    distance = 1000000000000;
    for k = 1:col_of_data
        temp = norm(object(:,j) - reference(:,k));
        if(distance>temp)
            aimone = k;distance = temp;
        end
    end
    if ceil(j/3)==ceil(aimone/4)
       num = num + 1;
    end
end
accuracy = num/col_of_test;msgbox(['       分类器准确率: ',num2str(accuracy)],'识别率');




% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8


% --- Executes during object creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton15.
%%CNN
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
        demo_CNN_MNIST;
        
% --- Executes on button press in pushbutton16.
%% 特征提取
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%     cla(handles.axes1);
%     cla(handles.axes2);
%     cla(handles.axes3);
    global im
    global ima1 ima2
    a=get(handles.popupmenu9, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('请选择实现方式','出错');
            end
        case 2
            im = rgb2gray(im);
            axes( handles.axes2 );
            imshow(im);
            im = double(im);
            keyPoints = SIFT(im,3,5,1.3);
            im = SIFTKeypointVisualizer(im,keyPoints);
            axes( handles.axes3 );
            imshow(uint8(im));
        case 3
            ima1=rgb2gray(ima1);
            ima1_Points = detectSURFFeatures(ima1);
            %打开axes1的句柄 进行axes1的操作
            axes(handles.axes1);
            imshow(ima1),hold on
            plot(selectStrongest(ima1_Points, 100));
            ima2=rgb2gray(ima2);
            %打开axes2的句柄 进行axes2的操作
            axes(handles.axes2);
            imshow(ima2),hold on
            ima2_Points = detectSURFFeatures(ima2);
            plot(selectStrongest(ima2_Points, 300));
            [ima1_Features, ima1_Points] = extractFeatures(ima1, ima1_Points);
            [ima2_Features, ima2_Points] = extractFeatures(ima2, ima2_Points);
            boxPairs = matchFeatures(ima1_Features, ima2_Features);
            matchedBoxPoints = ima1_Points(boxPairs(:, 1), :);
            matchedScenePoints = ima2_Points(boxPairs(:, 2), :);
            %%打开axes3的句柄 进行axes3的操作
            axes(handles.axes3);
            showMatchedFeatures(ima1,ima2, matchedBoxPoints,matchedScenePoints, 'montage');
            
    end

% --- Executes on selection change in popupmenu9.
function popupmenu9_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu9 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu9


% --- Executes during object creation, after setting all properties.
function popupmenu9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
