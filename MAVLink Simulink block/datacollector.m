function varargout = datacollector(varargin)
% DATACOLLECTOR MATLAB code for datacollector.fig
%      DATACOLLECTOR, by itself, creates a new DATACOLLECTOR or raises the existing
%      singleton*.
%
%      H = DATACOLLECTOR returns the handle to a new DATACOLLECTOR or the handle to
%      the existing singleton*.
%
%      DATACOLLECTOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DATACOLLECTOR.M with the given input arguments.
%
%      DATACOLLECTOR('Property','Value',...) creates a new DATACOLLECTOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before datacollector_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to datacollector_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help datacollector

% Last Modified by GUIDE v2.5 08-Feb-2014 00:14:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @datacollector_OpeningFcn, ...
                   'gui_OutputFcn',  @datacollector_OutputFcn, ...
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


% --- Executes just before datacollector is made visible.
function datacollector_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to datacollector (see VARARGIN)

% Choose default command line output for datacollector
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes datacollector wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = datacollector_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Plus.
function Plus_Callback(hObject, eventdata, handles)
% hObject    handle to Plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in Collect.
function Collect_Callback(hObject, eventdata, handles)
% hObject    handle to Collect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
