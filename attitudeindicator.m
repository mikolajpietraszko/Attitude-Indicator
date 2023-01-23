%%Attitude indicator 
%estabilishing serial communication 
%LLR_SW_17
clear;
serialportlist("available")'
arduinoobj = serialport("COM7", 115200);
configureTerminator(arduinoobj, "CR/LF");
%LLR_SW_18
arduinoObj.Timeout = 30;
flush(arduinoobj);
%LLR_SW_19
dataFrame = [1 1 1];
%flight instruments
%LLR_SW_20
f = uifigure;
artificialhorizon = uiaerohorizon(f);
artificialhorizon.Position = [50 50 350 350];
 
for i = 1:1000 %LLR_SW_21
flush(arduinoobj);
%LLR_SW_22
ASCIIdata = readline(arduinoobj);
SplitData = regexp(ASCIIdata,':','split');
dataFrame = str2double(SplitData);
artificialhorizon.Value = [dataFrame(2) dataFrame(1)];
%LLR_SW_23
temperature = [dataFrame(3)];
bar(temperature,'r');
ylim([0 50]);
disp(artificialhorizon.Value);
%LLR_SW_24
warnings(dataFrame);
end
