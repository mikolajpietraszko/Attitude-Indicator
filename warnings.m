function warnings(dataFrame) %LLR_SW_24
%PITCH ANGLE WARNINGS
if dataFrame(1)>35
    warning('Pitch angle too high!')
elseif dataFrame(1)<-35
    warning('Pitch angle too low!PULL UP!')
end
%ROLL ANGLE WARNINGS
if dataFrame(2)>35
    warning('Roll angle too high!')
elseif dataFrame(2)<-35
    warning('Roll angle too high!')
end
%TEMPERATURE WARNINGS
if dataFrame(3)>25
    warning('Temperature too high!')
elseif dataFrame(3)<5
    warning('Temperature too low!')
end
end