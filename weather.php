<?php
$device = $_REQUEST["device"];
$method = $_REQUEST["method"];
if(!$device||!$method) die("device or method param missing\n");


// add access token
$_REQUEST["access_token"] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

// create params
$params = http_build_query($_REQUEST);

// make request
$url = "https://api.spark.io/v1/devices/" . $device . "/" . $method . "?" . $params;
$response = file_get_contents($url);
echo $response; 
?>
