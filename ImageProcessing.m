function [] = ImageProcessing()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


end

function [array,counter] = add2Array(array,i,j)

    includedInArray = false;
    for c = 1 : counter
        if array{counter} == [i,j]
            includedInArray = true;
            c = counter;
        end
    end

    if includedInArray == false
        array = {array{:},[i,j]}
    end

end

