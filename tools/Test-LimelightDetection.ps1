<#
.SYNOPSIS
  Measures how reliably a Limelight detects AprilTags, without a robot.

.DESCRIPTION
  Polls the Limelight's built-in web API (port 5807, /results) for a number of seconds,
  counts how many camera frames saw a tag, and prints a summary:
    - detection rate (% of frames with a tag)
    - longest dropout (seconds in a row with no tag)
    - average tag area (bigger = more pixels on the tag)
    - camera FPS and temperature
  Each run is appended to bench-results.csv next to this script, so you can compare
  settings / tags / distances side by side.

  Setup: power the Limelight (12 V supply or PoE injector), plug it into your laptop with
  Ethernet, and make sure you can open http://limelight.local:5801 in a browser.

  NOTE: written from the Limelight API documentation; it has not been run against your
  camera yet. If it errors, check the address and that the web UI loads.

.EXAMPLE
  .\Test-LimelightDetection.ps1 -Label "6ft new tag downscale2"

.EXAMPLE
  .\Test-LimelightDetection.ps1 -Address 10.47.11.11 -Seconds 60 -Label "10ft lights off"
#>
param(
    [string]$Address = "limelight.local",
    [int]$Seconds = 30,
    [string]$Label = "unlabeled",
    [int]$PollHz = 40
)

$ErrorActionPreference = "Stop"
$baseUrl = "http://${Address}:5807"

function Get-LimelightResults {
    $response = Invoke-RestMethod -Uri "$baseUrl/results" -TimeoutSec 2
    # Older firmware wraps everything in a "Results" object
    if ($response.PSObject.Properties.Name -contains "Results") {
        return $response.Results
    }
    return $response
}

Write-Host "Connecting to $baseUrl ..."
try {
    $status = Invoke-RestMethod -Uri "$baseUrl/status" -TimeoutSec 3
    Write-Host "Connected. Status:" ($status | ConvertTo-Json -Compress)
} catch {
    Write-Host "Could not reach $baseUrl/status : $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "Check that http://${Address}:5801 opens in a browser. Try the IP (e.g. 10.47.11.11) instead of limelight.local."
    exit 1
}

Write-Host "Sampling for $Seconds seconds. Do not touch the camera or the tag..."
$frames = 0
$framesWithTag = 0
$areaSum = 0.0
$lastTimestamp = $null
$currentDropout = 0.0
$longestDropout = 0.0
$lastFrameTime = $null
$tagIdsSeen = @{}
$errors = 0

$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
while ($stopwatch.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $r = Get-LimelightResults
    } catch {
        $errors++
        Start-Sleep -Milliseconds (1000 / $PollHz)
        continue
    }

    # Only count NEW camera frames (we poll faster than the camera runs)
    $timestamp = $r.ts
    if ($null -ne $timestamp -and $timestamp -ne $lastTimestamp) {
        $lastTimestamp = $timestamp
        $now = $stopwatch.Elapsed.TotalSeconds
        $frames++

        $fiducials = @($r.Fiducial)
        $hasTag = ($r.v -eq 1) -or ($fiducials.Count -gt 0 -and $null -ne $fiducials[0])
        if ($hasTag) {
            $framesWithTag++
            $areaSum += [double]$r.ta
            foreach ($f in $fiducials) {
                if ($null -ne $f) { $tagIdsSeen[[int]$f.fID] = $true }
            }
            $currentDropout = 0.0
        } elseif ($null -ne $lastFrameTime) {
            $currentDropout += ($now - $lastFrameTime)
            if ($currentDropout -gt $longestDropout) { $longestDropout = $currentDropout }
        }
        $lastFrameTime = $now
    }
    Start-Sleep -Milliseconds (1000 / $PollHz)
}

try { $statusEnd = Invoke-RestMethod -Uri "$baseUrl/status" -TimeoutSec 3 } catch { $statusEnd = $null }

$detectionRate = 0.0
if ($frames -gt 0) { $detectionRate = 100.0 * $framesWithTag / $frames }
$avgArea = 0.0
if ($framesWithTag -gt 0) { $avgArea = $areaSum / $framesWithTag }
$measuredFps = $frames / $Seconds
$ids = ($tagIdsSeen.Keys | Sort-Object) -join " "

Write-Host ""
Write-Host "================ RESULT: $Label ================"
Write-Host ("Frames received      : {0}  ({1:N1} per second)" -f $frames, $measuredFps)
Write-Host ("Frames with a tag    : {0}" -f $framesWithTag)
Write-Host ("DETECTION RATE       : {0:N1} %" -f $detectionRate) -ForegroundColor Cyan
Write-Host ("Longest dropout      : {0:N2} s" -f $longestDropout)
Write-Host ("Average tag area     : {0:N3} % of image" -f $avgArea)
Write-Host ("Tag IDs seen         : {0}" -f $ids)
Write-Host ("HTTP errors          : {0}" -f $errors)
if ($null -ne $statusEnd) { Write-Host "Camera status at end:" ($statusEnd | ConvertTo-Json -Compress) }
Write-Host ""
if ($frames -eq 0) {
    Write-Host "No frames counted. The results JSON may not include 'ts' on this firmware; open $baseUrl/results in a browser and check." -ForegroundColor Yellow
} elseif ($detectionRate -lt 95) {
    Write-Host "Below 95% with a still camera and still tag = a detection problem (tag, lighting, exposure, focus). See docs/01-limelight-bench-test.md." -ForegroundColor Yellow
} else {
    Write-Host "Solid detection at this setup." -ForegroundColor Green
}

# Append to CSV for comparing runs
$csvPath = Join-Path $PSScriptRoot "bench-results.csv"
$row = [PSCustomObject]@{
    Time             = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
    Label            = $Label
    Seconds          = $Seconds
    Frames           = $frames
    Fps              = [math]::Round($measuredFps, 1)
    DetectionPercent = [math]::Round($detectionRate, 1)
    LongestDropoutS  = [math]::Round($longestDropout, 2)
    AvgTagArea       = [math]::Round($avgArea, 3)
    TagIds           = $ids
}
$row | Export-Csv -Path $csvPath -Append -NoTypeInformation
Write-Host "Saved to $csvPath"
