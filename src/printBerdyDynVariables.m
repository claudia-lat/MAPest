function printBerdyDynVariables( berdy )
% BERDYMEASUREMENTSWRAPPING orders the measurements in a format
% compatible with the BerdyHelper class.  It returns a vector of
% ordered measurements and its associated covariance matrix.

dynVariable = berdy.getDynamicVariablesOrdering();

for i = 1:size(dynVariable,2)
    currentInfo = dynVariable{i};
    
    type = currentInfo.type;
    typeStr = '';
    switch(type) 
        case 0
            typeStr = '6D acc      ';
        case 1
            typeStr = 'Net Wrench  ';
        case 2
            typeStr = 'Joint Wrench';
        case 3
            typeStr = 'Joint torque';
        case 4
            typeStr = 'Ext. Wrench ';
        case 5
            typeStr = 'Joint acc   ';
    end
    range = currentInfo.range;
    
    fprintf('[%d]Var type %s - id %s -- (idx-length) = (%d-%d)\n', i, typeStr, currentInfo.id, range.offset, range.size);
end
end

