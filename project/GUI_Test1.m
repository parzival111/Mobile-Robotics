function varargout = GUI_Test1(varargin)
% GUI_TEST1 MATLAB code for GUI_Test1.fig
%      GUI_TEST1, by itself, creates a new GUI_TEST1 or raises the existing
%      singleton*.
%
%      H = GUI_TEST1 returns the handle to a new GUI_TEST1 or the handle to
%      the existing singleton*.
%
%      GUI_TEST1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_TEST1.M with the given input arguments.
%
%      GUI_TEST1('Property','Value',...) creates a new GUI_TEST1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_Test1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_Test1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_Test1

% Last Modified by GUIDE v2.5 06-Feb-2020 22:24:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_Test1_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_Test1_OutputFcn, ...
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


% --- Executes just before GUI_Test1 is made visible.
function GUI_Test1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_Test1 (see VARARGIN)

% Choose default command line output for GUI_Test1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes GUI_Test1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_Test1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% connect to the arduino
delete(instrfind({'Port'},{'COM6'}));
s = serial('COM6');
fopen(s);
x = get(handles.edit1,'String');
fprintf("Data: " + x + "\n")


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map13_Callback(hObject, eventdata, handles)
% hObject    handle to map13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map13 as text
%        str2double(get(hObject,'String')) returns contents of map13 as a double


% --- Executes during object creation, after setting all properties.
function map13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map12_Callback(hObject, eventdata, handles)
% hObject    handle to map12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map12 as text
%        str2double(get(hObject,'String')) returns contents of map12 as a double


% --- Executes during object creation, after setting all properties.
function map12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map11_Callback(hObject, eventdata, handles)
% hObject    handle to map11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map11 as text
%        str2double(get(hObject,'String')) returns contents of map11 as a double


% --- Executes during object creation, after setting all properties.
function map11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map14_Callback(hObject, eventdata, handles)
% hObject    handle to map14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map14 as text
%        str2double(get(hObject,'String')) returns contents of map14 as a double


% --- Executes during object creation, after setting all properties.
function map14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map23_Callback(hObject, eventdata, handles)
% hObject    handle to map23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map23 as text
%        str2double(get(hObject,'String')) returns contents of map23 as a double


% --- Executes during object creation, after setting all properties.
function map23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map22_Callback(hObject, eventdata, handles)
% hObject    handle to map22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map22 as text
%        str2double(get(hObject,'String')) returns contents of map22 as a double


% --- Executes during object creation, after setting all properties.
function map22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map21_Callback(hObject, eventdata, handles)
% hObject    handle to map21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map21 as text
%        str2double(get(hObject,'String')) returns contents of map21 as a double


% --- Executes during object creation, after setting all properties.
function map21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map24_Callback(hObject, eventdata, handles)
% hObject    handle to map24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map24 as text
%        str2double(get(hObject,'String')) returns contents of map24 as a double


% --- Executes during object creation, after setting all properties.
function map24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map33_Callback(hObject, eventdata, handles)
% hObject    handle to map33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map33 as text
%        str2double(get(hObject,'String')) returns contents of map33 as a double


% --- Executes during object creation, after setting all properties.
function map33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map32_Callback(hObject, eventdata, handles)
% hObject    handle to map32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map32 as text
%        str2double(get(hObject,'String')) returns contents of map32 as a double


% --- Executes during object creation, after setting all properties.
function map32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map31_Callback(hObject, eventdata, handles)
% hObject    handle to map31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map31 as text
%        str2double(get(hObject,'String')) returns contents of map31 as a double


% --- Executes during object creation, after setting all properties.
function map31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map34_Callback(hObject, eventdata, handles)
% hObject    handle to map34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map34 as text
%        str2double(get(hObject,'String')) returns contents of map34 as a double


% --- Executes during object creation, after setting all properties.
function map34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map43_Callback(hObject, eventdata, handles)
% hObject    handle to map43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map43 as text
%        str2double(get(hObject,'String')) returns contents of map43 as a double


% --- Executes during object creation, after setting all properties.
function map43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map42_Callback(hObject, eventdata, handles)
% hObject    handle to map42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map42 as text
%        str2double(get(hObject,'String')) returns contents of map42 as a double


% --- Executes during object creation, after setting all properties.
function map42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map41_Callback(hObject, eventdata, handles)
% hObject    handle to map41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map41 as text
%        str2double(get(hObject,'String')) returns contents of map41 as a double


% --- Executes during object creation, after setting all properties.
function map41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function map44_Callback(hObject, eventdata, handles)
% hObject    handle to map44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of map44 as text
%        str2double(get(hObject,'String')) returns contents of map44 as a double


% --- Executes during object creation, after setting all properties.
function map44_CreateFcn(hObject, eventdata, handles)
% hObject    handle to map44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in mapSend.
function mapSend_Callback(hObject, eventdata, handles)
% hObject    handle to mapSend (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openArduino();
sendMap();
closeArduino();


% --- Executes on button press in refreshMap.
function refreshMap_Callback(hObject, eventdata, handles)
% hObject    handle to refreshMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function PATH_Callback(hObject, eventdata, handles)
% hObject    handle to PATH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PATH as text
%        str2double(get(hObject,'String')) returns contents of PATH as a double


% --- Executes during object creation, after setting all properties.
function PATH_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PATH (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goalY_Callback(hObject, eventdata, handles)
% hObject    handle to goalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goalY as text
%        str2double(get(hObject,'String')) returns contents of goalY as a double


% --- Executes during object creation, after setting all properties.
function goalY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goalX_Callback(hObject, eventdata, handles)
% hObject    handle to goalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goalX as text
%        str2double(get(hObject,'String')) returns contents of goalX as a double


% --- Executes during object creation, after setting all properties.
function goalX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posX_Callback(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posX as text
%        str2double(get(hObject,'String')) returns contents of posX as a double


% --- Executes during object creation, after setting all properties.
function posX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posY_Callback(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posY as text
%        str2double(get(hObject,'String')) returns contents of posY as a double


% --- Executes during object creation, after setting all properties.
function posY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in sendGoal.
function sendGoal_Callback(hObject, eventdata, handles)
% hObject    handle to sendGoal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in resetIR.
function resetIR_Callback(hObject, eventdata, handles)
% hObject    handle to resetIR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
openArduino();
i(1) = zeros(4,1);
while i(1) == 0
    i(1) = fscanf(s)
end
while i(2) == 0
    i(2) = fscanf(s)
end
while i(3) == 0
    i(3) = fscanf(s)
end
while i(4) == 0
    i(4) = fscanf(s)
end
handle.irF = i(1);
handle.irR = i(2);
handle.irB = i(3);
handle.irL = i(4);

% --- Executes on button press in sendPath.
function sendPath_Callback(hObject, eventdata, handles)
% hObject    handle to sendPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in sendPos.
function sendPos_Callback(hObject, eventdata, handles)
% hObject    handle to sendPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Custom Functions
function openArduino()
delete(instrfind({'Port'},{'COM6'}));
s = serial('COM6');
fopen(s);

function closeArduino()
fclose(s);

function sendMap()
openArduino();
fprintf(str2dbl(handle.map11,'String'));
fprintf(str2dbl(handle.map12,'String'));
fprintf(str2dbl(handle.map13,'String'));
fprintf(str2dbl(handle.map14,'String'));
fprintf(str2dbl(handle.map21,'String'));
fprintf(str2dbl(handle.map22,'String'));
fprintf(str2dbl(handle.map23,'String'));
fprintf(str2dbl(handle.map24,'String'));
fprintf(str2dbl(handle.map31,'String'));
fprintf(str2dbl(handle.map32,'String'));
fprintf(str2dbl(handle.map33,'String'));
fprintf(str2dbl(handle.map34,'String'));
fprintf(str2dbl(handle.map41,'String'));
fprintf(str2dbl(handle.map42,'String'));
fprintf(str2dbl(handle.map43,'String'));
fprintf(str2dbl(handle.map44,'String'));
closeArduino();




