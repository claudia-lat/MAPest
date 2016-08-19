function [] = saveURDF( URDFmodel,subjectID)
%SAVEURDF saves a file .urdf

fileID = fopen(sprintf('models/XSensURDF_subj%d.urdf',subjectID),'w');
fprintf(fileID,'%s', URDFmodel);
fclose(fileID);
end
