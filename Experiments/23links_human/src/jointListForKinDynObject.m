function jointListForKinDynObject(kinDyn)

    model = kinDyn.getRobotModel();
    
    for i = 0 : model.getNrOfJoints() -1
        disp(model.getJointName(i));
    end    
end
