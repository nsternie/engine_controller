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
objects = ["InputPhotos/HSol.png", "InputPhotos/VSol.png", "InputPhotos/Tank1.png", "InputPhotos/Tank2.png","InputPhotos/Dewar.png"];

%How many objects the code should find. Use same order as above
objectQuantity = [11, 7, 3, 2, 2];

%Some objects with more complex geometry benefit from having their origin
%specified as just the topmost pixel, with no regard to how far left or
%right it is. A 0 in the below array indicated topmost-leftmost while a 1
%specifies just topmost.

originPosArray = [0, 0, 1, 1, 1];

%While the best origin is auto detected by the computer, a user specified
%one is provided graphically for drawing later in python. This info will
%save and only be refreshed when the user desires.


shouldPullFromSave = input("Load Previous Object Origin Data? 1 OR 0: ");
if shouldPullFromSave == 1
    load('OriginData.mat');
else
    userOriginRowPosArray = [];
    userOriginColPosArray = [];
end

%Since the object might not be centered when only setting the origin to the
%topmost pixel, this array keeps track of the col offset. -1 means the
%object is set to topmost-leftmost.

originColOffset = [];

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
    
    %Displays how many pixels are in the object.
    %USE FOR THE THRESHOLD BELOW
    disp(['Man Number of Pixels in Object: ', num2str(sum(sum(logicalObjPosArray == 0)))])
    
    %Displays image with above cuttoff applied
    imshow(logicalObjPosArray)
    
    if shouldPullFromSave == 1
        title(strcat("Edited:  ", objects(s), "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
        goOn = input("Continue Execution? 1 or 0 ");
        if goOn == 0
            close all;
            return
        end
        close all;
    else
        title(strcat("Edited:  ", objects(s), "  **ZOOM IN. PRESS ENTER WHEN READY**"));
        zoom on;
        input("ZOOM IN. PRESS ENTER WHEN READY");
        zoom off;
        title(strcat("Edited:  ", objects(s), "  **SELECT PIXEL. ENTER x2 WHEN READY OR 0**"));
        %User zooms in image then picks pixel
        [colPix , rowPix, p] = impixel()
        userOriginRowPosArray(s) = rowPix(1);
        userOriginColPosArray(s) = colPix(1);
        goOn = input("Continue Execution? 1 or 0 ");
        if goOn == 0
            close all;
            return
        end
        close all;
    end
    %Gets the [row, column] cord of the topmost-leftmost pixel of the
    %object
    [rowOffset, columnOffset] = find(logicalObjPosArray(:, :) == 0);
    
    %Sets the Object Height and Width
    objectHeight(s) = max(rowOffset) - min(rowOffset) + 1;
    objectWidth(s) = max(columnOffset) - min(columnOffset) + 1;
    
    [rowMinOffset, rowI] = min(rowOffset);
    [columnMinOffset, colI] = min(columnOffset);
    
    
    %Shifts the position of the object to the specified origin pos for the
    %object
    if originPosArray(s) == 0
        logicalObjPosArray = circshift(logicalObjPosArray, [-rowMinOffset + 1, -columnMinOffset + 1]);
        originColOffset(s) = -1;
        if shouldPullFromSave == 0
            userOriginRowPosArray(s) = -(rowMinOffset - userOriginRowPosArray(s))
            userOriginColPosArray(s) = -(columnMinOffset - userOriginColPosArray(s))
        end
    elseif originPosArray(s) == 1
        logicalObjPosArray = circshift(logicalObjPosArray, [-rowMinOffset + 1, -columnOffset(rowI) + 1]);
        originColOffset(s) = columnOffset(rowI) - columnMinOffset;
        if shouldPullFromSave == 0
            userOriginRowPosArray(s) = -(rowMinOffset - userOriginRowPosArray(s))
            userOriginColPosArray(s) = -(columnOffset(rowI) - userOriginColPosArray(s))
        end
    end
    
    %Displays image with above cuttoff applied
    imshow(logicalObjPosArray)
    title(strcat("Edited:  ", objects(s), "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
    goOn = input("Continue Execution? 1 or 0 ");
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

%Save data if it is new
if shouldPullFromSave == 0
    save('OriginData.mat', 'userOriginRowPosArray', 'userOriginColPosArray');
end




%% FIND OBJECT CORDS
%This section of the code overlays the HSV Object Images ontop of a image
%full of the objects are various locations. When it finds a match it saves
%the location. 

%RGB & HSV B&W Image with various objects
fullObjImgName = 'InputPhotos/GUIBlackPNG.png';
rgbFullObjImg = imread(fullObjImgName);
hsvFullObjImg = rgb2hsv(rgbFullObjImg);

%RGB & HSV Colored Image with various Objects
fullColoredObjImgName = 'InputPhotos/GUIColoredPNG.png';
rgbColoredObjImg = imread(fullColoredObjImgName);
hsvColoredObjImg = rgb2hsv(rgbColoredObjImg);

%Creates a logical array for the position of the objects in the image.
%See %%CONSTANTS for more info
logicalFullObjPosArray = hsvFullObjImg(:,:,3) > hsvValueCutoff;

%Displays the image with the above cutoff applied
imshow(logicalFullObjPosArray);
title(strcat("Edited:  ", fullObjImgName, "  **PRESS ENTER TO CONTINUE OR 0 TO END** "));
goOn = input("Continue Execution? 1 or 0 ");
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
tresNumObjPixelMatchs = [110, 110, 500, 220, 220];

%Arrays to store xCord and yCord of where overlaps occur
xCord = [];
yCord = [];

%Array to store object type for given cords. 
objectType = [];

%Array to store the object color.
objectColor = [];

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
           disp(['Row: ', num2str(row),'     Col: ', num2str(col), '     Number of Obj Matches: ', num2str(numObjMatches), '     Number of Pixel Matches: ', num2str(numObjPixelMatchs)])
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
            if numObjPixelMatchs(s) > tresNumObjPixelMatchs(s)% && sum(sum(ismember(ismember(xCord, col-2:col+2) + ismember(yCord, row-2:row+2), 2))) == 0
                
                %Increase number of matches
                numObjMatches(s) = numObjMatches(s) + 1;
                
                %This block matches the color of the object to user
                %presets. It then asigns that object a color id.
                %It also marks the just found origin of the object with the
                %color it found.
                objectColorId = -1;
                
                nrow = userOriginRowPosArray(s) + row;
                ncol = userOriginColPosArray(s) + col;
                
                if hsvColoredObjImg(row,col,1) == 0 && hsvColoredObjImg(row, col,2) == 0
                    objectColorId = 0;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 1) = hsvColoredObjImg(nrow,ncol,1);
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 2) = 0;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 3) = 0;
                elseif hsvColoredObjImg(row,ncol,1) == 0 && hsvColoredObjImg(row,ncol,2) > .1
                    objectColorId = 1;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 1) = hsvColoredObjImg(nrow,ncol,1);
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 2) = 1;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 3) = hsvColoredObjImg(nrow,ncol,3);
                elseif hsvColoredObjImg(row,col,1) == .5
                    objectColorId = 2;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 1) = hsvColoredObjImg(nrow,ncol,1);
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 2) = 1;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 3) = hsvColoredObjImg(nrow,ncol,3);
                elseif hsvColoredObjImg(row,col,1) > .25 && hsvColoredObjImg(row,col,1) < .29
                    objectColorId = 3;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 1) = hsvColoredObjImg(nrow,ncol,1);
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 2) = 1;
                    hsvColoredObjImg(nrow-1: nrow+1,ncol-1: ncol+1 , 3) = hsvColoredObjImg(nrow,ncol,3);
                end
              
                
                %Add Cord to arrays and set object type value
                xCord = [xCord ncol];
                yCord = [yCord nrow];
                objectType = [objectType, s];
                objectColor = [objectColor, objectColorId];
                
                %Removes the just matched object from the full object
                %image. This is important for optimization because this
                %declutters the image and allows the program to skip moving
                %back over pixels it has already encountered and matched.
                if originPosArray(s) == 0
                    logicalFullObjPosArray(row - 2: row + objectHeight(s) + 2, col - 2:col + objectWidth(s)+ 2) = 1;
                elseif originPosArray(s) == 1
                    col - (objectHeight(s) - (objectHeight(s) - originColOffset(s))) - 2 
                    logicalFullObjPosArray(row - 2: row + objectHeight(s) + 2, col - (objectWidth(s) - (objectWidth(s) - originColOffset(s))) - 2 : col + (objectWidth(s) - originColOffset(s)) + 2) = 1;
                end
                
                %Variable to keep track that an object was matched this
                %loop
                justFound(s) = 1;
                
                %Show the image with blue dots for where the matched
                %objects origin is.
                figure(1)
                imshow(hsv2rgb(hsvColoredObjImg));
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
        disp(['Row: ', num2str(row),'     Col: ', num2str(col), '     Number of Obj Matches: ', num2str(numObjMatches), '     Number of Pixel Matches: ', num2str(numObjPixelMatchs)])
        
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
csvOutputMatrix = [objectType - 1; objectColor; xCord; yCord];

%Name of csvFile
csvFileName = "csvObjectData.csv";

%Write the Csv
csvwrite(csvFileName, csvOutputMatrix);









