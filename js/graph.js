// Graph
// Sliding / Rolling Graph - shows set duration of data points

function Graph (spec) {
  this.rangeX = spec.rangeX; // we expect duration of time in seconds
  this.maxY = spec.maxY;
  this.minY = spec.minY;
  this.maxX = spec.maxX ? spec.maxX : spec.rangeX;
  this.minX = spec.minX ? spec.minX : 0;
  this.svg = spec.svg ? spec.svg : false;
  this.lines = spec.lines ? spec.lines : [];
  // duration in ms between graph redraws (e.g. 100ms)
  this.period = spec.period;
}

// Graph.init() add initialization
Graph.prototype.init = function (elemId) {
  var margin = {top: 20, right: 20, bottom: 30, left: 50},
      width = window.innerWidth - margin.left - margin.right,
      height = window.innerHeight - margin.top - margin.bottom;

  var that = this,
      minY = this.minY,
      maxY = this.maxY;

  // domain === range of values for x-axis, range === length of x-axis on page
  this.x = d3.scale.linear()
    .domain([this.minX, this.maxX])
    .range([0, width]);

  // create x axis
  this.xAxis = d3.svg.axis()
    .scale(this.x)
    .orient("bottom");

  this.y = d3.scale.linear()
    .domain([minY, maxY])
    .range([height, 0]);

  var yAxis = d3.svg.axis()
    .scale(this.y)
    .orient("left");

  //  create line function x(i) scales i to x-axis
  this.lineModel = d3.svg.line()
    .x(function(d, i) { return that.x(d.x); })
    .y(function(d, i) { return that.y(d.y); });

  // make svg if not provided
  if (!this.svg) {
    this.svg = d3.select("#" + elemId)
    .append("svg")
      .attr("width", width + margin.left + margin.right)
      .attr("height", height + margin.top + margin.bottom)
    .append("g")
      .attr("transform", "translate(" + margin.left + "," + margin.top + ")");
  }

  // add x axis
  this.svg.append("g")
    .attr("class", "x_axis")
    .attr("transform", "translate(0," + height + ")")
    .call(this.xAxis);

  // add y axis
  this.svg.append("g")
    .attr("class", "y axis")
    .call(yAxis);
}

Graph.prototype.startDrawing = function (period) {
  var that = this,
    elapsedTime = 0,
    graphPoints = [];

  setInterval(function(){
    var line;

    /* UPDATE X-AXIS */

    // update x min and max
    that.minX = elapsedTime - that.rangeX;
    that.maxX = elapsedTime;



    // update graph x axis
    that.x.domain([that.minX, that.maxX]);
    that.xAxis.scale(that.x);

    // update x axis
    d3.selectAll(".x_axis").call(that.xAxis);


    /* UPDATE LINES */

    // loop through lines
    for (var i = 0; i < that.lines.length; i++) {
      line = that.lines[i];

      graphPoints = [];

      /*
       *  only graph lines where the x is greater than minX axis
       *  find first point where x is greater than minX
       *  graph that point and all newer points in array
       */
      for (var j = line.minDataIndex; j < line.data.length; j++) {

        if (line.data[j].x > that.minX) {
          line.minDataIndex = j;
          graphPoints = line.data.slice(j);
          break;
        }
      }

      // is there a way to update lines by moving them left?
      d3.selectAll("." + line.className).remove();

      // draw line
      that.svg.append("path")
        .datum(graphPoints)
        .attr("class", line.className)
        .attr("d", that.lineModel);
    }

    // update elapsed time -> convert to seconds
    elapsedTime += that.period / 1000; //
  }, this.period)
}


/* ----- Line Stuff ----- */


function Line (spec) {
  this.className = spec.className;
  this.data = spec.data ? spec.data : [];
  this.minDataIndex = 0;
}

Line.prototype.startTracking = function (period) {
  // is there a better way to accomplish this / that?
  setInterval(this.generateData(period), period);
}


// var l = new Line({
//   className: "line1"
// });
//
// l.startTracking(100);
//
// var l2 = new Line({
//   className: "line2"
// });
//
// l2.startTracking(500);
//
// var g = new Graph({
  // rangeX: 10, // we expect duration of time in seconds
  // maxY: 10,
  // minY: 0
// });
//
// g.init();
//
// g.lines.push(l);
// g.lines.push(l2);
//
// g.startDrawing(10);
