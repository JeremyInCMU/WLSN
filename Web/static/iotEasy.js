var selTabIndex = 0;
var prev_Tab = 'entryway';

/*
 *  Manipulate the user interface
 */

function selectTab(prevTab, selTab) {
    var selTabObj = $('.' + selTab),
        prevTabObj = $('.' + prevTab),
        prevTabObjText = prevTabObj.text(),
        selTabObjText = selTabObj.find('a').text() || prevTabObjText;

    var link = $('<a/>', {href: "#", text: prevTabObjText});
    var span = $('<span/>', {text: selTabObjText});

    if (prevTab) {
    prevTabObj.empty().append(link);
    }
    selTabObj.empty().append(span);

    $('.screenContainer,.screen').each(function () {
        id = $(this).attr('id');
        if (id != selTab){
            $(this).css({'left': '1200px'});
            $(this).css({'visibility': 'hidden'});
            $(this).attr({'aria-hidden': 'true'});
            $('#' + id).find('ul.hotspotlist').css({'display': 'none', 'visibility': 'hidden'});
        } else {
            $(this).css({'left': '-100px'});
            $(this).css({'display': 'block', 'visibility': 'visible'});
            $(this).removeAttr('aria-hidden');
            $('#' + selTab).find('ul.hotspotlist').css({'display': 'block', 'visibility': 'visible'});
        }
        prev_Tab = selTab;
    });
    console.log(prev_Tab);
}

/*
 *  Define an object: data
 */

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

/*
 *  Global variable
 */
var text = '';


function submitFile(event) {
    //var uploadFile = "/Users/Jeremy/Desktop/Data.txt";
    var uploadFile = document.getElementById("selectfile");
    var f = uploadFile.files[0];
    var r = new FileReader();
    r.readAsText(f);
    r.onload = function (evt) {
        text = evt.target.result;
        console.log(r.readyState);
    };
}


function transmit(event){
    var dataStrings = text.split("\n"); // change the split symbol
    console.log(dataStrings);

    for (var i= 0; i<dataStrings.length;i++) {
        var dataString = dataStrings[i];
        var attributes = dataString.split(',');

        var newdata = new Data(attributes[1], attributes[0], attributes[2],
            +attributes[3], attributes[4], attributes[5],
            +attributes[6], attributes[7], attributes[8]);

        // check value of each sensor; if outliers sending email to user
        var sendData = "?NodeID=" + attributes[1] + "&Time=" + attributes[0]
            + "&Accelerometer=" + attributes[2] + "&Pressure=" + attributes[3]
            + "&Light=" + attributes[4] + "&Sound=" + attributes[5]
            + "&Motion=" + attributes[6] + "&Humidity=" + attributes[7]
            + "&Thermometer=" + attributes[8];
        $.ajax({
            url: "/addData/" + sendData, async:false,success: function (result) {
                console.log(result);
                sensorCheck(newdata);
            }
        });
    }
}


var dataSet=[];

/*
 *  Periodically acquire data from database to website
 *  These two helper functions for the Sensor Data Part
 */

// init data to time selection
function initDataToTimeSelection(){
    $.ajax({
        url:"/initTimeData/", success:function(result){
            var dataStrings = result.split(";");
            for (var k=0; k< dataStrings.length;k++){
                var dataString = dataStrings[k];
                var attributeStrings = dataString.split(",");
                addDataToTimeSelection(attributeStrings[1]);
            }
        }
    })
}

// add data to time selection
function addDataToTimeSelection(new_data){
    var timeList = document.getElementById('Time');
    // some undeterminated things here, check if new_data is a function or an object
    if (typeof(new_data) == 'object') {
        timeList.add(new Option(new_data.Time));
    }
    else{
        console.log('ok');
        timeList.add(new Option(new_data));
    }
}

// it is self-executed and periodical
var intervalID = window.setInterval(dataAcquisition, 5000);

function dataAcquisition(event){
    $.ajax({
        url:"/dataAcquisition/",success:function(result){
            var dataStrings = result.split(";");
            for (var i = 0; i < dataStrings.length;i++){
                var dataString = dataStrings[i];
                var attributeStrings = dataString.split(",");
                if (attributeStrings[0]== document.getElementById("NodeID").value &&
                    attributeStrings[1]== document.getElementById("Time").value){
                    var newData = new Data( attributeStrings[0],attributeStrings[1], attributeStrings[2],
                        attributeStrings[3], attributeStrings[4], attributeStrings[5], attributeStrings[6],
                        attributeStrings[7], attributeStrings[8]);
                    switch(document.getElementById('Sensor-input').value){
                        case 'Battery':
                            document.getElementById('sensorVal').value = newData.Battery;
                            break;

                        case 'Accelerometer':
                            document.getElementById('sensorVal').value = newData.Accelerometer;
                            break;

                        case 'Pressure Sensor':
                            document.getElementById('sensorVal').value = newData.Pressure;
                            break;

                        case 'Light Sensor':
                            document.getElementById('sensorVal').value = newData.Light;
                            break;

                        case 'Sound Sensor':
                            document.getElementById('sensorVal').value = newData.Sound;
                            break;

                        case 'Motion Sensor':
                            document.getElementById('sensorVal').value = newData.Motion;
                            break;

                        case 'Humidity Sensor':
                            document.getElementById('sensorVal').value = newData.Humidity;
                            break;

                        case 'Thermometer':
                            document.getElementById('sensorVal').value = newData.Thermometer;
                            break;
                    }
                    var isNewData = true;
                    for(var j=0; j< dataSet.length; j++){
                        if(newData.NodeID == dataSet[j].NodeID || newData.Time == dataSet[j].Time){
                            isNewData = false;
                        }
                    }
                    if(isNewData == true){
                        dataSet.push(newData);
                        console.log(dataSet);
                        sensorCheck(newData);
                    }
                }
            }
            alert("A total of " +  dataSet.length  + " data points are added to the array.");
            }
    })
}

/*
 * set Threshod
 */
function Threshod(ba=100,ac=200,ps=300,ls=400,ss=500,ms=600,hs=700,ts=800){
    this.Battery = ba;
    this.Accelerometer = ac;
    this.Pressure = ps;
    this.Light = ls;
    this.Sound = ss;
    this.Motion = ms;
    this.Humidity = hs;
    this.Thermometer = ts;
}

var sensorThreshod = new Threshod();

function setThreshod(event){
    switch(document.getElementById('Sensor-input-threshod-sensortype')){
        case 'Battery':
            sensorThreshod.Battery = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Accelerometer':
            sensorThreshod.Accelerometer = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Pressure Sensor':
            sensorThreshod.Pressure = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Light Sensor':
            sensorThreshod.Light = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Sound Sensor':
            sensorThreshod.Sound = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Motion Sensor':
            sensorThreshod.Motion = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Humidity Sensor':
            sensorThreshod.Humidity = document.getElementById('Sensor-input-threshod-value').value;
            break;

        case 'Thermometer':
            sensorThreshod.Thermometer = document.getElementById('Sensor-input-threshod-value').value;
            break;
    }
}


/*
 * check if there is outlier. if there is, server will send email to user
 */
function sensorCheck(new_data){

    var command = '?Command=' + '';
    if (Number(new_data.Accelerometer) > sensorThreshod.Accelerometer)
    {
    command += new_data.NodeID + " " + "Accelerometer has a problem";
    }
    else if(new_data.Pressure > sensorThreshod.Pressure)
    {
        command += new_data.NodeID + " " + "Pressure sensor has a problem"
    }
    else if(new_data.Light < sensorThreshod.Light)
    {
        command += new_data.NodeID + " " +  "Light sensor has a problem"
    }
    else if(new_data.Sound < sensorThreshod.Sound)
    {
        command += new_data.NodeID + " " + "Sound sensor has a problem"
    }
    else if(new_data.Motion > sensorThreshod.Motion)
    {
        command += new_data.NodeID + " " + "Motion sensor has a problem"
    }
    else if(new_data.Humidity > sensorThreshod.Humidity)
   {
        command += new_data.Humidity+ " " + "Humidity sensor has a problem"
   }
    else if(new_data.Thermometer > sensorThreshod.Thermometer)
    {
        command += new_data.Thermometer + " " + "Thermometer sensor has a problem"
    }

    $.ajax({
        url:"/sendEmail/" + command, async:false,success: function (result){
            console.log(result);
        }
    });
}

/*
 *  Draw topology
 */

var preRouteList = '';
//var count = 0;

function DrawNodes(event) {
    var sys = arbor.ParticleSystem(1, 1, 1);
    sys.parameters({gravity: true});
    sys.renderer = Renderer("#viewport");
    // define gateway and nodes
    var data = [];
    var gateway = sys.addNode('GateWay', {'size': 10, 'color': 'red', 'shape': 'dot', 'label': 'GateWay'});
    var node1 = sys.addNode('Node1', {'size': 5, 'color': 'blue', 'shape': 'dot', 'label': 'Node1'});
    var node2 = sys.addNode('Node2', {'size': 5, 'color': 'green', 'shape': 'dot', 'label': 'Node2'});
    var node3 = sys.addNode('Node3', {'size': 5, 'color': 'black', 'shape': 'dot', 'label': 'Node3'});
    var node4 = sys.addNode('Node4', {'size': 5, 'color': 'gray', 'shape': 'dot', 'label': 'Node4'});
    var node5 = sys.addNode('Node5', {'size': 5, 'color': 'purple', 'shape': 'dot', 'label': 'Node5'});
    var node6 = sys.addNode('Node6', {'size': 5, 'color': 'green', 'shape': 'dot', 'label': 'Node6'});
    var node7 = sys.addNode('Node7', {'size': 5, 'color': 'brown', 'shape': 'dot', 'label': 'Node7'});
    var node8 = sys.addNode('Node8', {'size': 5, 'color': 'yellow', 'shape': 'dot', 'label': 'Node8'});

    data.push(gateway);
    data.push(node1);
    data.push(node2);
    data.push(node3);
    data.push(node4);
    data.push(node5);
    data.push(node6);
    data.push(node7);
    data.push(node8);

    var routeList = '';
    //if(count%2==0) {
    //    routeList = 'Node1,Node2';
    //}
    //else{
    //    routeList = 'Node2,Node3';
    //}
    //count+=1;

    if (routeList != preRouteList){
        //var canvas = document.getElementById("#viewpoint");
        //var context = canvas.getContext('2d');
        var nodes = routeList.split(',');
        for (i = 0; i < nodes.length -1; i++) {
            for (j=0; j< data.length; j++){
                if (data[j].name == nodes[i]){
                    console.log('ok');
                    sys.addEdge(data[j], data[j + 1])
                }
            }
        }
    }
}


$(document).ready(function(){
    $(".entryway").click(openEntryWay = function(){
        var selectedTab = 'entryway';
        selectTab(prev_Tab,selectedTab);
    });
    $(".living").click(openLivingRoom = function(){
        var selectedTab = 'living';
        selectTab(prev_Tab,selectedTab);
    });
    $(".kitchen").click(openKitchen = function(){
        var selectedTab = 'kitchen';
        selectTab(prev_Tab,selectedTab);
    });
    $(".laundry").click(openLaundry = function(){
        var selectedTab = 'laundry';
        selectTab(prev_Tab,selectedTab);
    });
    $("#submit").click(submitFile);
    $("#transmit").click(transmit);
    $("#NodeID").change(dataAcquisition);
    $("#Time").change(dataAcquisition);
    $("#Sensor-input").change(dataAcquisition);
    $('#setThreshod').click(setThreshod);
    DrawNodes();
    initDataToTimeSelection();
});

