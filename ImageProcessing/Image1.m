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
set(gca,'XColor',get(gca,'Color')) ;% �����д��빦�ܣ��������������̶�תΪ��ɫ
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % �����д��빦�ܣ�ȥ������̶�
set(gca,'YTickLabel',[]);
% Hint: place code in OpeningFcn to populate axes1


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'XColor',get(gca,'Color')) ;% �����д��빦�ܣ��������������̶�תΪ��ɫ
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % �����д��빦�ܣ�ȥ������̶�
set(gca,'YTickLabel',[]);
% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(gca,'XColor',get(gca,'Color')) ;% �����д��빦�ܣ��������������̶�תΪ��ɫ
set(gca,'YColor',get(gca,'Color'));
 
set(gca,'XTickLabel',[]); % �����д��빦�ܣ�ȥ������̶�
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
    global im %����һ��ȫ�ֱ���  
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'����ͼ��');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('û��ѡ���ļ�','����');
    return;
    else 
    str=[pathname,filename];
    end
    %%��ͼ��  
    im=imread(str);  
    %%��axes1�ľ�� ����axes1�Ĳ���  
    axes(handles.axes1);  
    %%��axes1����ʾԭʼͼ��  
    imshow(im);  
    


% --- Executes on button press in pushbutton2.
%% ��������
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu1, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('��ѡ��ʵ�ַ�ʽ','����');
            end
        case 2
            im1= rgb2gray(im);%�Ҷȴ���
            im2=imnoise(im1,'salt & pepper',0.04);%��ӽ�������
            %%��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            %%��axes2����ʾ�Ҷ�ͼ��
            imshow(im2),xlabel('����ͼ��','Fontsize',12);
            im3 = medfilt2(im2,[2,4]);%ȥ�봦��
            %%��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            %%��axes2����ʾȥ���ͼ��
            imshow(im3);
        case 3
            im1= rgb2gray(im);%�Ҷȴ���
            %%��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            %%��axes2����ʾ�Ҷ�ͼ��
            imshow(im1);
            im2 = histeq(im1);
            %%��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            %%��axes2����ʾֱ��ͼ��
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
%% ��Ե���
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu2, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('��ѡ��ʵ�ַ�ʽ','����');
            end
        case 2
            im1= rgb2gray(im);
            %��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            %��axes2����ʾ�Ҷ�ͼ��
            imshow(im1);
            %ʹ��canny���ӽ��б�Ե����
            im1=double(im1);%��ͼ��ת��Ϊdouble��
            thresh=[0.02,0.2];%ͨ�������Ҷ�ͼ�е㣬��ͼ����Ϣ��ֵ������������ͼƬֻ�ж���ɫֵ��
            sigma=2;%ָ��sigma�����˲����ı�׼ƫ��
            im2=edge(im1,'canny',thresh,sigma);
            %%��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            %%��axes3����ʾ��Եͼ��
            imshow(im2,[]);
        case 3
            im1= rgb2gray(im);
            %ʹ��canny���ӽ��б�Ե����
            im1=double(im1);
            thresh=[0.02,0.2];
            sigma=2;
            im2=edge(im1,'canny',thresh,sigma);
            %%��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            %%��axes2����ʾ��Եͼ��
            imshow(im2,[]);
            [H,theta,rho]=hough(im2,'RhoResolution',0.5,'ThetaResolution',0.5);
            peak=houghpeaks(H,5);%��ֵ��
            lines=houghlines(im2,theta,rho,peak);%����ԭͼֱ����Ϣ
            %��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            %��axes3����ʾ�Ҷ�ͼ��
            imshow(im2),hold on
            for k=1:length(lines)
                xy=[lines(k).point1;lines(k).point2];
                max_len=0;
                len=norm(lines(k).point1-lines(k).point2);
                if(len>max_len)
                    xy_long=xy;
                    %�����ߵ������յ�
                    plot(xy(1,1),xy(1,2),'xw','LineWidth',2,'Color','y');%���
                    plot(xy(2,1),xy(2,2),'xw','LineWidth',2,'Color','r');%�յ�
                    plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','c');%�����߶�
                end
            end
    end

% --- Executes on button press in pushbutton5.
%% ͼ��ָ�
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global im
    a=get(handles.popupmenu4, 'value');
    switch a
        case 1
            if real(a)==1
                errordlg('��ѡ��ʵ�ַ�ʽ','����');
            end
        case 2
            im=rgb2gray(im);%ת��Ϊ�Ҷ�ͼ
            %��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            imshow(im);
            I_double=double(im);
            [wid,len]=size(im);%ͼ��Ĵ�С
            %�Ҷȼ�
            colorLevel=256;
            %ֱ��ͼ
            hist=zeros(colorLevel,1);
            %����ֱ��ͼ
            for i=1:wid
                for j=1:len
                    m=im(i,j)+1;%ͼ��ĻҶȼ�m
                    hist(m)=hist(m)+1;%�Ҷ�ֵΪi�����غ�
                end
            end
            %ֱ��ͼ��һ��
            hist=hist/(wid*len);%���Ҷ�ֵ���� Pi
            miuT=0;%���������ֵ
            for m=1:colorLevel
                miuT=miuT+(m-1)*hist(m);  %�����ֵ
            end
            xigmaB2=0;%
            for mindex=1:colorLevel
                threshold=mindex-1;%�趨��ֵ
                omega1=0;%Ŀ�����
                for m=1:threshold-1
                    omega1=omega1+hist(m);% Ŀ����� W0
                end
                omega2=1-omega1; %�����ĸ��� W1
                miu1=0;%Ŀ���ƽ���Ҷ�ֵ
                miu2=0;%������ƽ���Ҷ�ֵ
                for m=1:colorLevel
                    if m<threshold
                        miu1=miu1+(m-1)*hist(m);%Ŀ�� i*pi���ۼ�ֵ[1 threshold]
                    else
                        miu2=miu2+(m-1)*hist(m);%���� i*pi���ۼ�ֵ[threshold m]
                    end
                end
                miu1=miu1/omega1;%Ŀ���ƽ���Ҷ�ֵ
                miu2=miu2/omega2;%������ƽ���Ҷ�ֵ
                xigmaB21=omega1*(miu1-miuT)^2+omega2*(miu2-miuT)^2;%��󷽲�
                xigma(mindex)=xigmaB21;%���趨һ��ֵ �ٱ������лҶȼ�
                %�ҵ�xigmaB21��ֵ���
                if xigmaB21>xigmaB2
                    finalT=threshold;%�ҵ���ֵ �Ҷȼ�
                    xigmaB2=xigmaB21;%����Ϊ���
                end
            end
            %��ֵ��һ��
            for i=1:wid
                for j=1:len
                    if I_double(i,j)>finalT %�������趨�ľ�ֵ ��ΪĿ��
                        bin(i,j)=0;
                    else
                        bin(i,j)=1;
                    end
                end
            end
            %��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            imshow(bin);
        case 3            
            % ת��Ϊ�Ҷ�ͼ����
            gray=rgb2gray(im);
            [m,n]=size(gray);
            k=3; % �������
            % ��ͼ�����RGB����3ͨ���ֽ�
            A = reshape(im(:, :, 1), m*n, 1);
            B = reshape(im(:, :, 2), m*n, 1);
            C = reshape(im(:, :, 3), m*n, 1);
            data = [A B C];                         
            % ����ͼ��
            res = kmeans(double(data), k);
            result = reshape(res, m, n);             % ����ת��ΪͼƬ��ʽ
            %��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            % label2rgb������ת����Ǿ���RGBͼ��
            imshow(label2rgb(result)),
            title(strcat('K=',num2str(k),'ʱRGBͨ���ָ���'));   % ��ʾ�ָ���           
            % ������
            res = kmeans(double(data), k+2);
            result = reshape(res, m, n);
            %��axes3�ľ�� ����axes3�Ĳ���
            axes(handles.axes3);
            imshow(label2rgb(result)),
            title(strcat('K=',num2str(k+2),'ʱRGBͨ���ָ���')); % ��ʾ�ָ���

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
%% ѡ���һ��ͼ
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    cla(handles.axes1);
    cla(handles.axes2);
    cla(handles.axes3);
    global ima1
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'����ͼ��');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('û��ѡ���ļ�','����');
    return;
    else 
    str=[pathname,filename];
    end
    %%��ͼ��  
    ima1=imread(str);  
    %%��axes1�ľ�� ����axes1�Ĳ���  
    axes(handles.axes1);  
    %%��axes1����ʾԭʼͼ��  
    imshow(ima1),hold on  

% --- Executes on button press in pushbutton8.
%% ѡ��ڶ���ͼ
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global ima2
    [filename,pathname]=uigetfile({'*.jpg';'*.bmp';'*.tif';'*.*'},'����ͼ��');
    if isequal(filename,0)||isequal(pathname,0)
       errordlg('û��ѡ���ļ�','����');
    return;
    else 
    str=[pathname,filename];
    end
    %%��ͼ��  
    ima2=imread(str);
    %%��axes2�ľ�� ����axes2�Ĳ���  
    axes(handles.axes2);  
    %%��axes2����ʾԭʼͼ��  
    imshow(ima2);


% --- Executes on button press in pushbutton9.
%% Ŀ����
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
      global im
        a=get(handles.popupmenu8, 'value');
        switch a
            case 1
                if real(a)==1
                    errordlg('��ѡ��ʵ�ַ�ʽ','����');
                end
            case 2
                I1=rgb2gray(im);%�����ǽ����ɫͼ��ת��Ϊ�Ҷ�ͼ�񣬼��ҶȻ�����
                %%��axes2�ľ�� ����axes2�Ĳ���
                axes(handles.axes2);
                %%��axes2����ʾԭʼͼ��
                imshow(I1);
                I2=edge(I1,'Prewitt',0.15,'both');
                %�����ǲ���I1��Ϊ�������룬������һ����I1��ͬ��С�Ķ�ֵ��ͼ��I2��
                %�ں�����⵽��Ե�ĵط�Ϊ1�������ط�Ϊ0
                se=[1;1;1];
                I3=imerode(I2,se);%��ʴ
                se=strel('rectangle',[25,25]);
                I4=imclose(I3,se);
                I5=bwareaopen(I4,2000);%������ɾ����ֵͼ��BW�����С��2000�Ķ���
                [y,x]=size(I5);
                myI=double(I5);%double����
                Blue_y=zeros(y,1);%zeros�����Ƿ���һ��m��n��p��...��double�������
                for i=1:y
                    for j=1:x
                        if(myI(i,j,1)==1)
                            
                            Blue_y(i,1)= Blue_y(i,1)+1;%��ɫ���ص�ͳ��
                        end
                    end
                end
                [~, MaxY]=max(Blue_y);%Y����������ȷ��
                PY1=MaxY;
                while ((Blue_y(PY1,1)>=5)&&(PY1>1))
                    PY1=PY1-1;
                end
                PY2=MaxY;
                while ((Blue_y(PY2,1)>=5)&&(PY2<y))
                    PY2=PY2+1;
                end
                %%%%%% X���� %%%%%%%%%
                Blue_x=zeros(1,x);%��һ��ȷ��x����ĳ�������
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
                PX1=PX1-1;%�Գ��������У��
                PX2=PX2+1;
                dw=im(PY1:PY2-8,PX1:PX2,:);
                bh=rgb2gray(dw);%�����ǽ����ɫͼ��ת��Ϊ�Ҷ�ͼ�񣬼��ҶȻ�����
                %%��axes3�ľ�� ����axes3�Ĳ���
                axes(handles.axes3);
                %%��axes2����ʾԭʼͼ��
                imshow(bh);
            case 3
                im = rgb2gray(im);
                % ������ֵ
                bw = imbinarize(im,0.69);%��ֵ��
                % ͨ�������ж��ֶ�ȥ��
                [m,n] = size(bw);
                for i = 2:m-1
                    for j = 2:n-1
                        %ͬ����Ԫ���ж�
                        if(bw(i,j)~=bw(i+1,j) && bw(i,j)~=bw(i-1,j))
                            bw(i,j) = 1;
                            %ͬ����Ԫ���ж�
                        elseif(bw(i,j)~=bw(i,j+1) && bw(i,j)~=bw(i,j-1))
                            bw(i,j) = 1;
                            %ͬб��Ԫ���ж�
                        elseif(bw(i,j)~=bw(i+1,j+1) && bw(i,j)~=bw(i-1,j-1))
                            bw(i,j) = 1;
                            %ͬб��Ԫ���ж�
                        elseif(bw(i,j)~=bw(i-1,j+1) && bw(i,j)~=bw(i+1,j-1))
                            bw(i,j) = 1;
                        end
                    end
                end
                
                % ��ֵ��ͼ��ȡ��
                for i = 1:m
                    for j = 1:n
                        bw(i,j) = ~bw(i,j);
                    end
                end
                %��axes2�ľ�� ����axes2�Ĳ���
                axes(handles.axes2);
                imshow(bw);
                bw = bwareaopen(bw,30);%ɾ����ֵͼ��BW�����С��bw�Ķ���Ĭ�������ʹ��8����
                % ͼ��ѧ�ṹԪ�ع�����Բ�ΰ뾶Ϊ8
                se = strel('disk',8);
                % �ز���
                bw = imclose(bw,se);
                % ���׶�
                bw = imfill(bw,'holes');
                [B,L] = bwboundaries(bw,'noholes');
                %��axes3�ľ�� ����axes3�Ĳ���
                axes(handles.axes3);
                imshow(label2rgb(L,@jet,[.5 .5 .5])),hold on;
                for k = 1:length(B)
                    boundary = B{k};
                    % ��ʾ��ɫ�߽�
                    plot(boundary(:,2),boundary(:,1),'w','LineWidth',2)
                end
                % ȷ��Բ��Ŀ��
                stats = regionprops(L,'Area','Centroid');
                % ���������
                threshold = 0.85;%������ֵ
                for k = 1:length(B)
                    boundary = B{k};
                    delta_sq = diff(boundary).^2;
                    % ���ܳ�
                    perimeter = sum(sqrt(sum(delta_sq,2)));
                    % �����
                    area = stats(k).Area;
                    metric = 4*pi*area/perimeter^2;
                    metric_string = sprintf('%2.2f',metric);
                    % ������ֵƥ��
                    if metric > threshold
                        centroid = stats(k).Centroid;
                        plot(centroid(1),centroid(2),'ko');
                        text(centroid(1)-2,centroid(2)-2, '����Բ��','Color',...
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
%% PCA����ʶ��
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

        % ������ȡָ���ļ����µ�ͼƬ128*128
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
        % ���Ļ� & ����Э�������
        imgmean = mean(imagedata,2);
        for i = 1:col_of_data
            imagedata(:,i) = imagedata(:,i) - imgmean;
        end
        covMat = imagedata'*imagedata;
        [COEFF, latent, explained] = pcacov(covMat);
        % ѡ�񹹳�95%����������ֵ
        i = 1; proportion = 0;
        while(proportion < 95)
            proportion = proportion + explained(i);
            i = i+1;
        end
        p = i - 1; 
        % ������
        W = imagedata*COEFF;    % N*M��
        W = W(:,1:p);           % N*p��
        % ѵ����������������µı����� p*M
        reference = W'*imagedata;

% --- Executes on button press in pushbutton12.
%% PCA����ʶ��
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
%% PCA����ʶ��
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

% Ԥ����������
ima = double(ima(:));
objectone = W'*(ima - imgmean);
distance = 100000000;

% ��С���뷨��Ѱ�Һʹ�ʶ��ͼƬ��Ϊ�ӽ���ѵ��ͼƬ
for k = 1:col_of_data
    temp = norm(objectone - reference(:,k));
    if(distance>temp)
        aimone = k;
        distance = temp;
        % ��ʾ���Խ��
        aimpath = strcat(pathname, '/', img_path_list(aimone).name);
        axes( handles.axes2 );
        imshow(aimpath);
    end
end

% --- Executes on button press in pushbutton14.
%% PCA����ʶ��
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ѡ����Լ�

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
% ��С���뷨��Ѱ�Һʹ�ʶ��ͼƬ��Ϊ�ӽ���ѵ��ͼƬ�� ���������׼ȷ��
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
accuracy = num/col_of_test;msgbox(['       ������׼ȷ��: ',num2str(accuracy)],'ʶ����');




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
%% ������ȡ
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
                errordlg('��ѡ��ʵ�ַ�ʽ','����');
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
            %��axes1�ľ�� ����axes1�Ĳ���
            axes(handles.axes1);
            imshow(ima1),hold on
            plot(selectStrongest(ima1_Points, 100));
            ima2=rgb2gray(ima2);
            %��axes2�ľ�� ����axes2�Ĳ���
            axes(handles.axes2);
            imshow(ima2),hold on
            ima2_Points = detectSURFFeatures(ima2);
            plot(selectStrongest(ima2_Points, 300));
            [ima1_Features, ima1_Points] = extractFeatures(ima1, ima1_Points);
            [ima2_Features, ima2_Points] = extractFeatures(ima2, ima2_Points);
            boxPairs = matchFeatures(ima1_Features, ima2_Features);
            matchedBoxPoints = ima1_Points(boxPairs(:, 1), :);
            matchedScenePoints = ima2_Points(boxPairs(:, 2), :);
            %%��axes3�ľ�� ����axes3�Ĳ���
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
