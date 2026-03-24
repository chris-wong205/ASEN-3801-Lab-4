function saveAllOpenFigures(saveDir, fileFormat)
    % saveAllOpenFigures Saves all currently open MATLAB figures.
    %
    % Usage:
    %   saveAllOpenFigures() - Saves as PNGs in the current directory.
    %   saveAllOpenFigures('myFolder') - Saves as PNGs in 'myFolder'.
    %   saveAllOpenFigures('myFolder', 'pdf') - Saves as PDFs in 'myFolder'.

    % Set default arguments if none are provided
    if nargin < 1 || isempty(saveDir)
        saveDir = pwd; % Current working directory
    end
    if nargin < 2 || isempty(fileFormat)
        fileFormat = 'png'; % Default format
    end

    % Create the save directory if it doesn't exist
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end

    % Get handles to all open figures
    figs = findall(groot, 'Type', 'figure');

    if isempty(figs)
        disp('No open figures found to save.');
        return;
    end

    % Loop through each figure and save it
    for i = 1:length(figs)
        fig = figs(i);
        
        % Create a filename using the Figure's Name (if it has one) or Number
        if ~isempty(fig.Name)
            % Clean up the name so it is a valid file name
            safeName = matlab.lang.makeValidName(fig.Name); 
            fileName = sprintf('Figure_%d_%s.%s', fig.Number, safeName, fileFormat);
        else
            fileName = sprintf('Figure_%d.%s', fig.Number, fileFormat);
        end
        
        fullPath = fullfile(saveDir, fileName);
        
        % Save the figure
        try
            % Note: For R2020a or newer, exportgraphics(fig, fullPath) is 
            % an excellent alternative that crops whitespace perfectly.
            saveas(fig, fullPath); 
            fprintf('Saved: %s\n', fullPath);
        catch ME
            fprintf('Failed to save Figure %d: %s\n', fig.Number, ME.message);
        end
    end
    
    disp('Finished saving figures!');
end