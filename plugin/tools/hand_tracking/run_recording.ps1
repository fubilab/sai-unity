param(
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$Exe = "C:\FuBIntLab\repos\sai-unity\plugin\target\Release\main_depthai.exe",
    [string]$Models = "C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker\models"
)

# Timeline, seconds after the first color frame arrives.
$phases = @(
    @{ t = 0;  say = "Recording. Keep both hands out of view." },
    @{ t = 6;  say = "Show one open hand and move it slowly around the frame." },
    @{ t = 16; say = "Hold one hand still, about forty centimeters away." },
    @{ t = 20; say = "Now hold it still, about seventy centimeters away." },
    @{ t = 24; say = "Show both hands. Open palms." },
    @{ t = 28; say = "Make fists." },
    @{ t = 32; say = "Peace signs." },
    @{ t = 36; say = "Fast motion. Wave and rotate your palms, front and back." },
    @{ t = 44; say = "Move hands in and out of the frame edges." },
    @{ t = 52; say = "Done. Hands down." }
)
$runSeconds = 54

Add-Type -AssemblyName System.Speech
$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
$voice.Rate = 1

New-Item -ItemType Directory -Force $OutDir | Out-Null
Remove-Item "$OutDir\*" -Force -ErrorAction SilentlyContinue

$env:SAI_RECORD_DIR = $OutDir
$env:SAI_RUN_SECONDS = "$runSeconds"
$env:SAI_HAND_TRACKING_DEBUG = "1"
$stdout = "$OutDir\stdout.txt"
$stderr = "$OutDir\stderr.txt"

$p = Start-Process -FilePath $Exe `
    -ArgumentList "`"$Models\palm_detection_sh4.blob`"", "`"$Models\hand_landmark_lite_sh4.blob`"" `
    -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru

# Wait for the device to boot and the first color frame.
$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline) {
    if ($p.HasExited) { Write-Host "harness exited early"; exit 1 }
    if ((Test-Path $stdout) -and (Select-String -Path $stdout -Pattern "color frame" -Quiet)) { break }
    Start-Sleep -Milliseconds 200
}
Write-Host "first color frame seen; starting timeline"
$voice.Speak("Ready. Starting in three, two, one.")

$t0 = Get-Date
foreach ($ph in $phases) {
    while (((Get-Date) - $t0).TotalSeconds -lt $ph.t) { Start-Sleep -Milliseconds 50 }
    Write-Host ("[{0,5:N1}s] {1}" -f ((Get-Date) - $t0).TotalSeconds, $ph.say)
    [void]$voice.SpeakAsync($ph.say)
}

$null = $p.WaitForExit(15000)
if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
Write-Host "finished"
Get-ChildItem $OutDir | Select-Object Name, Length
