%%Jack Talierio
%jtalier@umich.edu
%MASA

clear;
close all;

%% CONSTANTS
%These values rule the whole program and can be editied here.
%Note that you have to go through the whole program and change the areas
%where specified. This is only a collection of some values to change. 

%At HSV Values below this number the program assumes that the pixel is
%white. If the returned Object Image is missing pixels raise this number.
%If it has too many pixels, lower this number. (Default .88)
hsvValueCutoff = .88;



%% SETUP
%This part of the code prepares the object images. It first converts to hsv
%and sets the for s = 1:imgSize(2)colord pixels to a value of 1. This works best for black
%images. It then gets the topmost and leftmost point (the origin) and moves 
%it to the topleft pixel of the image. This gets the object image ready for
%comparision.

%PNG Image of objects to look for. All images must be the exact same size.
%The object can be anywhere in the image. Low resolution is perfered for
%performance. ~~(1000 x 1000)
objects = ["SingleSolHor.png", "SingleSolVer.png"];

%How many objects the code should find. Use same order as above
objectQuantity = [11, 7];

%Estimation on the pixel Height and Width
objectHeight = [];
objectWidth = [];

%Final HSV Object Images
finalHSVObjImg = {};

%Gets how many object image files there are to read
[junk , objectImgQuantity] = size(objects);

%Runs through all elements in objects
for s = 1:objectImgQuantity
    
    %Get RGB and HSV image of object
    rgbObjImg = imread(objects(s));
    hsvObjImg = rgb2hsv(rgbObjImg);
    
    %Creates a logical array for the position of the object in the image.
    %See %%CONSTANTS for more info
    logicalObjPosArray = hsvObjImg(:,:,3) > hsvValueCutoff;
    
    %Displays image with above cuttoff applied
    imshow(logicalObjPosArray)
    title(strcat("Edited:  ", objects(s), "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
    goOn = input("Continue Execution? 1 or 0");
    if goOn == 0
        close all;
        return
    end
    close all;
    
    %Gets the [row, column] cord of the topmost-leftmost pixel of the
    %object
    [rowOffset, columnOffset] = find(logicalObjPosArray(:, :) == 0);
    
    %Sets the Object Height and Width
    objectHeight(s) = max(rowOffset) - min(rowOffset) + 1;
    objectWidth(s) = max(columnOffset) - min(columnOffset) + 1;
    
    rowOffset = min(rowOffset);
    columnOffset = min(columnOffset);
    
    
    %Shifts the position of the object to the topmost-leftmost pixel of the
    %image
    logicalObjPosArray = circshift(logicalObjPosArray, [-rowOffset + 1, -columnOffset + 1]);
    
    %Displays image with above cuttoff applied
    imshow(logicalObjPosArray)
    title(strcat("Edited:  ", objects(s), "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
    goOn = input("Continue Execution? 1 or 0");
    if goOn == 0
        close all;
        return
    end
    close all;
    
    %Sets the original HSV Object Image's value channel to the shifted values
    hsvObjImg(:,:, 3) = logicalObjPosArray;
    
    %Adds HSV Object Image to the cell array.
    finalHSVObjImg{1, s} = hsvObjImg; 
end




%% MATCHING
%This section of the code overlays the HSV Object Images ontop of a image
%full of the objects are various locations. When it finds a match it saves
%the location. 

%RGB & HSV Image with various objects
fullObjImgName = 'SolToCordPNG.png';
rgbFullObjImg = imread(fullObjImgName);
hsvFullObjImg = rgb2hsv(rgbFullObjImg);

%Creates a logical array for the position of the objects in the image.
%See %%CONSTANTS for more info
logicalFullObjPosArray = hsvFullObjImg(:,:,3) > hsvValueCutoff;

%Displays the image with the above cutoff applied
imshow(logicalFullObjPosArray);
title(strcat("Edited:  ", fullObjImgName, "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
goOn = input("Continue Execution? 1 or 0");
if goOn == 0
    close all;
    return
end
close all;

%Get the images row and col size
[imgsRowSize, imgsColSize] = size(logicalFullObjPosArray);


%This is a messy for loop structure that could certainly be optimized. It
%works by taking the object images starting at the topmost-leftmost pixel
%and working their ways left->right, top->bottom. At each position it
%compares the amount of black pixels overlapping. If their is a certain
%quantity met (defined by user below) then the progam assumes a match and
%logs the cordinate data. This continues for the entire image. So far the
%optimixation that has been included is: removing matched objects from the 
%master image, skipping rows that have no black pixels, skiiping to the 
%first non black pixel in a row, and moving to the next col if the rest of
%the row is empty.

%Contains number of matched objects. Is in same order as object array
%above.
numObjMatches = zeros(size(objects));

%Contains number of pixels that match between HSV Obj Img and the Full Obj 
%Img. Is in same order as object array above.
numObjPixelMatchs = zeros(size(objects));

%Threshold for the # of pixel matches required for the program to consider
%it a match. User Determined. Is in same order as object array above.
tresNumObjPixelMatchs = [120, 100];

%Arrays to store xCord and yCord of where overlaps occur
xCord = [];
yCord = [];

%Array to store object type for given cords. 
objectType = [];


%Some global variables needed for the loop
col = 1;
nextBlackPixelIndex = 1;
justFound = zeros(size(objects));

%Iterates over a full row then moves down column
for row = 1:imgsRowSize
    col = 1;
    while (col <= imgsColSize)
        %If the entire row is devoid of black pixels skip it
        if sum(logicalFullObjPosArray(row,:) == 0) == 0
           disp([row, col, numObjMatches, numObjPixelMatchs])
           break
        end
        
        %This for loops checks each object against the full image and gets
        %the # of pixel matches
        for s = 1:objectImgQuantity
            
            %Gets the HSV Object Image
            hsvObjImg = finalHSVObjImg{s};
            
            %Pixel Matches = the sum of all the times where the black
            %pixels are overlapping
            numObjPixelMatchs(s) = sum(sum(logicalFullObjPosArray == 0 & logicalFullObjPosArray == hsvObjImg(:,:,3)));
            
            %If the current number of pixel matches is greater than the
            %treshold AND there is not another suspected match +-2 pixels
            %away THEN record the cords of the match.
            if numObjPixelMatchs(s) > tresNumObjPixelMatchs(s) && sum(sum(ismember(ismember(xCord, col-2:col+2) + ismember(yCord, row-2:row+2), 2))) == 0
                
                %Increase number of matches
                numObjMatches(s) = numObjMatches(s) + 1;
                
                %Add Cord to arrays and set object type value
                xCord = [xCord col];
                yCord = [yCord row];
                objectType = [objectType, s];
                
                %Sets the cordinate pixel colored for human verification
                %The cordinate is the center of the box
                hsvFullObjImg(row-3:row+3, col-3: col + 3, 1) = .7;
                hsvFullObjImg(row-3:row+3, col-3: col + 3, 2) = 1;
                hsvFullObjImg(row-3:row+3, col-3: col + 3, 3) = 1;
                
                %Removes the just matched object from the full object
                %image. This is important for optimization because this
                %declutters the image and allows the program to skip moving
                %back over pixels it has already encountered and matched.
                logicalFullObjPosArray(row - 2: row + objectHeight(s) + 2, col - 2:col + objectWidth(s)+ 2) = 1;
                
                %Variable to keep track that an object was matched this
                %loop
                justFound(s) = 1;
                
                %Show the image with blue dots for where the matched
                %objects origin is.
                figure(1)
                imshow(hsv2rgb(hsvFullObjImg));
                title("Full Obj Image with Colored Cord Pixels");
            else
                %Update to say not found
                justFound(s) = 0;
            end
            
            %Uncomment this to watch the program real times
            %RUNS SLOWER
            %figure(2)
            %Shsv(:,:,3) = logicalFullObjPosArray & hsvObjImg(:,:,3);
            %imshow(hsv2rgb(Shsv));
        end
        %Display info for user convience
        disp([row, col, numObjMatches, numObjPixelMatchs])
        
        %Checks where the next black pixel index is. If it is in this row
        %it skips to it, otherwise it moves to the next col.
        nextBlackPixelIndex = find(logicalFullObjPosArray(row, col+2:end) == 0, 1) + col;
        
        %If there is no black pixel break to next row.
        if sum(nextBlackPixelIndex) == 0
            break;
        end
        
        %Shift over the HSV Obj Values to the right every loop
        %It will always shift over to the next black pixel or next row;
        for s = 1:objectImgQuantity
            hsvObjImg = finalHSVObjImg{s};
            
            %Logic for when to auto shift
            if col == 1 || sum(sum(justFound)) > 0
                hsvObjImg(:,:,3) = circshift(hsvObjImg(:,:,3), nextBlackPixelIndex - col,2);
            else
                hsvObjImg(:,:,3) = circshift(hsvObjImg(:,:,3), 1,2);
            end
            
            finalHSVObjImg{s} = hsvObjImg;
        end
        
        %Update the col # for diffrent conditions
        if col == 1
            col = nextBlackPixelIndex;
        elseif sum(sum(justFound)) > 0
            col = nextBlackPixelIndex - 1;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %I know what you are thinking.   %
        %Why is this here like this?     %
        %Cant you just add the 1         %
        %right above? No you can't.      %
        %I do not know why but for       %
        %your sake do not try.           %
                                         %
        col = col + 1;                   %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    %Shift over the HSV Obj Values down by one every loop
    %Also make sure it is shifted to leftmost pixel
    for s = 1:objectImgQuantity
        hsvObjImg = finalHSVObjImg{s};
        %Shift Obj Image down
        hsvObjImg(:,:,3) = circshift(hsvObjImg(:,:,3),1,1);
        %Shift Obj image back to left side
        if col ~= 1
            hsvObjImg(:,:,3) = circshift(hsvObjImg(:,:,3), -col + 2,2);
        end
        
        finalHSVObjImg{s} = hsvObjImg;
    end
end

%Cleans up figures
if ishandle(figure(2))
    close(2);
end

%% OUTPUT

%Creates the CSV Output Matrix
csvOutputMatrix = [objectType - 1; xCord; yCord];

%Name of csvFile
csvFileName = "csvObjectData.csv";

csvwrite(csvFileName, csvOutputMatrix);









