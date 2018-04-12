function printBerdyDynVariables_floating( berdy )
% PRINTBERDYDYNVARIABLE_FLOATING prints the order of the vector d.  For each time
% frame, d is a vector column vector [berdy.getNrOfDynamicVariables() x 1]
% where to each ith-link it s associated the following structure:
%
%            d_i = [a_i, f_i, fx_i].
% 
% For each variable, the function returns:
% - the type of variable;
% - the index of its location in the vector d (NOTE: it is in  0-based
%   notation!);
% - the range or lenght of element for that variable.

dynVariable = berdy.getDynamicVariablesOrdering();

for i = 1:size(dynVariable,2)
    currentInfo = dynVariable{i};
    
    type = currentInfo.type;
    typeStr = '';
    switch(type) 
        case iDynTree.LINK_BODY_PROPER_CLASSICAL_ACCELERATION
            typeStr = '6D acc      ';
%         case iDynTree.NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV
%             typeStr = 'Net Wrench  ';
        case iDynTree.JOINT_WRENCH
            typeStr= 'Joint Wrench';
%         case iDynTree.DOF_TORQUE
%             typeStr = 'Joint torque';
        case iDynTree.NET_EXT_WRENCH
            typeStr = 'Ext. Wrench ';
%         case iDynTree.DOF_ACCELERATION
%             typeStr = 'Joint acc   ';
    end
    range = currentInfo.range;
    
    fprintf('[%d]Var type %s - id %s -- (idx-length) = (%d-%d)\n', i, typeStr,...
              currentInfo.id, range.offset, range.size);
end
end
