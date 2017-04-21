/**
 * Created by Jeremy on 3/17/16.
 */

$(document).ready(function(){
    $("#InitData").click(initData);
    $("#addDataInfoToList").click(addDataInfoToList);
    $("#Time").change(DataInfoListChange);
    window.onload = DrawGraph;
});

// define a prototype data
function Data(id,time,a,p,l,s,m,h,t){
    this.NodeID = id;
    this.Time = time;
    this.Accelerometer = a;
    this.Pressure = p;
    this.Light = l;
    this.Sound = s;
    this.Motion = m;
    this.Humidity = h;
    this.Thermometer = t;
}

// define data set
var dataSet = [];

// define function initData with ajax
function initData(){
    $.ajax({
        url:"/loadData/",success:function(result){
            var dataStrings = result.split(";");
            for (var i = 0; i < dataStrings.length;i++){
                var dataString = dataStrings[i];
                console.log(dataString);
                var attributeStrings = dataString.split(",");
                console.log(attributeStrings[0]);
                console.log(document.getElementById("NodeID").innerHTML);
                if (attributeStrings[0]== document.getElementById("NodeID").innerHTML) {
                    var newData = new Data( attributeStrings[0],attributeStrings[1], attributeStrings[2],
                        attributeStrings[3], attributeStrings[4], attributeStrings[5], attributeStrings[6],
                        attributeStrings[7], attributeStrings[8]);
                    dataSet.push(newData);
                    console.log(dataSet);
                }
            }
            alert("A total of" + dataSet.length + "data points are added to the array.");
            }
    })
}


function addDataInfoToList(){
    if(dataSet.length == 0){
        alert("Please load the initial drawings to the array first.");
        return;
    }
    // Empty the Time list before add list to it
    var Time = document.getElementById("Time");
    Time.innerHTML = "";
    console.log(dataSet);
    for (var i=0; i< dataSet.length;i++){
        //In order to add an item to the list, we need to
        //create a tag<option>, which defines an item in a <select> tag
        var newOption = document.createElement("option");
        newOption.innerText = dataSet[i].Time;
        newOption.value = i;

        Time.appendChild(newOption);
        DataInfoListChange();
    }
    alert("A total of" + Time.length + "time points are added to the HTML list" )
}

function DataInfoListChange(event){
    var Time = document.getElementById("Time");
    var TimeIndex = Time.selectedIndex;
    var selectedData = dataSet[TimeIndex];

    document.getElementById("Accelerometer").value = selectedData.Accelerometer;
    document.getElementById("Pressure").value = selectedData.Pressure;
    document.getElementById("Light").value = selectedData.Light;
    document.getElementById("Sound").value = selectedData.Sound;
    document.getElementById("Motion").value = selectedData.Motion;
    document.getElementById("Humidity").value = selectedData.Humidity;
    document.getElementById("Thermometer").value = selectedData.Thermometer;
}


function DrawGraph() {

    var dps = []; // dataPoints

    var chart = new CanvasJS.Chart("chartContainer",{
        title :{
            text: "Node Data"
        },
        data: [{
            type: "line",
            dataPoints: dps
        }]
    });

    var xVal = 0;
    var yVal = 100;
    var updateInterval = 100;
    var dataLength = 500; // number of dataPoints visible at any point

    var updateChart = function (count) {
        count = count || 1;
        // count is number of times loop runs to generate random dataPoints.

        for (var j = 0; j < count; j++) {
            yVal = yVal +  Math.round(5 + Math.random() *(-5-5));
            dps.push({
                x: xVal,
                y: yVal
            });
            xVal++;
        }
        if (dps.length > dataLength)
        {
            dps.shift();
        }

        chart.render();

    };

    // generates first set of dataPoints
    updateChart(dataLength);

    // update chart after specified time.
    setInterval(function(){updateChart()}, updateInterval);

}
