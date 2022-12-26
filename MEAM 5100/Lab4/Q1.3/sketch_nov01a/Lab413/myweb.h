const char body[] PROGMEM = R"===(

<!DOCTYPE html>
<html>
<body>
<div class="topnav">
        <h1>Lab 4.1.3</h1>
    </div>
    <div class="content">
        <div class="card-grid">
            <div class="card">
                <p class="card-title">LED Brightness</p>
                <p class="switch">
                    <input type="range"  id="slider1" min="0" max="100" step="1" value ="80" class="slider1">
                </p>
                <p class="state">Brightness: <span id="sliderValue1"></span> &percnt;</p>
            </div>
            <div class="card">
                <p class="card-title"> Frequency Control</p>
                <p class="switch">
                    <input type="range"  id="slider2" min="3" max="30" step="1" value ="20" class="slider2">
                </p>
                <p class="state">Frequency: <span id="sliderValue2"></span> Hz</p>
            </div>
            
        </div>
    </div>
</body>
<script>
slider1.onchange = function dutyFunction () {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("sliderValue1").innerHTML = this.responseText;
      }
    };
    var str = "slider1?val=";
    var res = str.concat(this.value);
    xhttp.open("GET", res, true);
    xhttp.send();
  }
    
  slider2.onchange = function freqFunction() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("sliderValue2").innerHTML = this.responseText;
      }
    };
    var str = "slider2?val=";
    var res = str.concat(this.value);
    xhttp.open("GET", res, true);
    xhttp.send();
  }

</script>
</html>


)===";
