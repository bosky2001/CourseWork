
const char body[] PROGMEM=R"===(
<!DOCTYPE html>
<html>
  <body>
    <a href="/L">Backwards </a> <br>
    
    <a href="/H">Forwards  </a> <br>
    
    
    <br>
    <input type="range" min="1" max="100" value="1"  id="slider">
    <span id="sliderlabel">  </span> <br>
  </body>
  <script>
    function hit() {
      var xhttp = new XMLHttpRequest();
      xhttp.open("GET", "hit", true);
      xhttp.send();
    }
    setInterval(updateLabel, 1000);
    updateLabel();
    function updateLabel() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("buttonlabel").innerHTML = this.responseText;
        }
      };
      xhttp.open("GET", "LEDstate", true);
      xhttp.send();
    }
    slider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("sliderlabel").innerHTML = this.responseText;
        }
      };
      var str = "slider?val=";
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }
  </script>
</html>
)===";
