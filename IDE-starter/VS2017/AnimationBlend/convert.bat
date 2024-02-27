@echo on
setlocal


set amcfile=""
set asffile=""


if /i "%~x1"==".amc" (
	set amcfile="%~dpnx1"
	set iskelfile="%~dp1import_skeleton-%~n1.mel"
	set imovfile="%~dp1import_mov-%~n1.mel"
	set movfile="%~dpn1.mov"
)


if /i "%~x2"==".amc" (
	set amcfile="%~dpnx2"
	set iskelfile="%~dp2import_skeleton-%~n2.mel"
	set imovfile="%~dp2import_mov-%~n2.mel"
	set movfile="%~dpn2.mov"
)


if /i "%~x1"==".asf" set asffile="%~dpnx1"
if /i "%~x2"==".asf" set asffile="%~dpnx2"




if %amcfile%=="" goto error
if %asffile%=="" goto error


%~dp0asf2mel -f %asffile% %iskelfile% 


if errorlevel 1 (
echo Error running asf2mel [see above]
pause
goto end
)


%~dp0amc2mov -f %asffile% %amcfile% 60 %imovfile% %movfile%


if errorlevel 1 (
echo Error running amc2mov [see above]
pause
goto end
)


goto end


:error
echo Error in command line. Please pass asf and amc files.

pause
:end
