class PlotDefinition:

    #initialize plot definition
    def __init__(self, coords, title='', xlabel='', ylabel='', data_range=100):
        self.data = []
        self.range = data_range #allows individualized ranges
        self.plots = []
        self.coords = coords
        self.title = title
        self.labels = {"left": ylabel, "bottom": xlabel}

    #set the reference to x data
    #only actually sets name of header in database
    #then call database[self.x] to get actual array of data
    def setX(self, xHeader):
        self.x = xHeader

    #add a y dataset
    #again its only a list with a reference and the line style
    #eventually this will compile the header to allow math operations
    def addY(self, tup):
        self.data.append(tup)

    #makes a PlotItem with proper labels and formatting and pushes it to GraphicsLayout
    def makePlot(self, plot_box, show_legend=True, show_grid=True):
        self.parent = plot_box.addPlot(row=self.coords[0], col=self.coords[1], title=self.title, labels=self.labels)
        if show_legend:
            self.parent.addLegend()
        if show_grid:
            self.parent.showGrid(x=True,y=True)
        for dataset in self.data:
            self.plots.append(self.parent.plot(name=dataset[2], pen=dataset[1]))

    #update the plot
    #this is why I used a class
    #once I set it up I never need to know what it plots again
    #I can just update it with the full database and it handles the rest
    def updatePlot(self, database):
        #cut data to length
        self.database = database.tail(self.range)

        #update each set of y data
        for i in range(len(self.plots)):
            self.plots[i].setData(self.database[self.x],self.database[self.data[i][0]])