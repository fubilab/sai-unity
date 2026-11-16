# Guided pinch recording: runs main_depthai with hand tracking (live, log + color frames recorded) and speaks a pose
# every ~11 s: an announcement, then a "hold" cue, so that the analysis window is after the reaction time.
# Needs the camera (close the Unity Editor) and one hand in view at a time. Writes hands.jsonl, phases.json, color.raw
# (about 1.7 GB) and vio.jsonl to OutDir; analyze with analyze_pinch.py OutDir.
# Usage: .\pinch_session.ps1 -OutDir C:\somewhere\pinch1
param(
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$Exe = "C:\FuBIntLab\repos\sai-unity\plugin\target\Release\main_depthai.exe",
    [string]$Models = "C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker\models"
)

# cls: pos = should be a pinch, neg = should not, cycle = pinch and release repeatedly, ask = ambiguous (OK sign)
$poses = @(
    @{ id = "open";          cls = "neg";   period = 11; say = "Open hand, facing the camera." },
    @{ id = "pinch_right";   cls = "pos";   period = 11; say = "Pinch with your right hand, palm facing the camera." },
    @{ id = "pinch_left";    cls = "pos";   period = 11; say = "Pinch with your left hand, palm facing the camera." },
    @{ id = "pinch_back";    cls = "pos";   period = 11; say = "Pinch with the back of your hand facing the camera." },
    @{ id = "pinch_side";    cls = "pos";   period = 11; say = "Pinch with your hand sideways, thumb side facing the camera." },
    @{ id = "pinch_near";    cls = "pos";   period = 11; say = "Pinch close to the camera, about thirty centimeters." },
    @{ id = "pinch_far";     cls = "pos";   period = 11; say = "Pinch far from the camera, about seventy centimeters." },
    @{ id = "pinch_cycle";   cls = "cycle"; period = 14; say = "Pinch and release slowly, three times." },
    @{ id = "pinch_move";    cls = "pos";   period = 14; say = "Pinch and move your hand slowly left and right, like carrying a small object." },
    @{ id = "fist";          cls = "neg";   period = 11; say = "Make a fist." },
    @{ id = "thumbs_up";     cls = "neg";   period = 11; say = "Thumbs up." },
    @{ id = "almost";        cls = "neg";   period = 11; say = "Thumb and index finger almost touching, with a small gap." },
    @{ id = "ok_sign";       cls = "ask";   period = 11; say = "O K sign: thumb and index finger in a circle, other fingers open." }
)
$leadIn = "Pinch recording. Place the device so that it faces you, and keep only one hand in view at a time. Starting in ten seconds."

Add-Type -AssemblyName System.Speech
$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
New-Item -ItemType Directory -Force $OutDir | Out-Null
Remove-Item "$OutDir\*" -Force -ErrorAction SilentlyContinue

foreach ($k in @("SAI_HAND_REPLAY", "SAI_HAND_NO_DEPTH", "SAI_USE_STEREO", "SAI_USE_FEATURE_TRACKER", "SAI_FAST_VIO", "SAI_USE_COLOR", "SAI_RECORD_NO_COLOR", "SAI_IR_PROJECTOR")) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
$total = 0
foreach ($ph in $poses) { $total += $ph.period }   # not Measure-Object: it cannot read keys of a hashtable
$env:SAI_RECORD_DIR = $OutDir
$env:SAI_HAND_LOG = "$OutDir\hands.jsonl"
$env:SAI_HAND_COLOR_FPS = "15"
$env:SAI_HAND_TRACKING_DEBUG = "1"
if ($total -lt 60) { throw "total period is $total" }
$env:SAI_RUN_SECONDS = [string]($total + 40)
Write-Host "Harness runs for $($env:SAI_RUN_SECONDS) s"

$p = Start-Process -FilePath $Exe -ArgumentList "`"$Models\palm_detection_sh4.blob`"", "`"$Models\hand_landmark_full_sh4.blob`"" `
    -RedirectStandardOutput "$OutDir\stdout.txt" -RedirectStandardError "$OutDir\stderr.txt" -PassThru

$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline) {
    if ($p.HasExited) { Write-Host "harness exited early"; exit 1 }
    if ((Test-Path "$OutDir\stdout.txt") -and (Select-String -Path "$OutDir\stdout.txt" -Pattern "hands:" -Quiet)) { break }
    Start-Sleep -Milliseconds 200
}
$firstFrameWall = Get-Date
$voice.Speak($leadIn)
$t0Wall = Get-Date

$meta = @{ offset_first_frame_to_t0 = ($t0Wall - $firstFrameWall).TotalSeconds; phases = @() }
$t = 0.0
foreach ($ph in $poses) {
    $start = $t
    while (((Get-Date) - $t0Wall).TotalSeconds -lt $start) { Start-Sleep -Milliseconds 30 }
    Write-Host ("[{0,6:N1}s] {1}" -f ((Get-Date) - $t0Wall).TotalSeconds, $ph.say)
    [void]$voice.SpeakAsync($ph.say)
    $holdAt = $start + $(if ($ph.cls -eq "cycle" -or $ph.id -eq "pinch_move") { 5.5 } else { 5.5 })
    while (((Get-Date) - $t0Wall).TotalSeconds -lt $holdAt) { Start-Sleep -Milliseconds 30 }
    [void]$voice.SpeakAsync($(if ($ph.cls -eq "cycle") { "Go." } else { "Hold." }))
    $meta.phases += @{ id = $ph.id; cls = $ph.cls; announce = $start; hold = $holdAt; end = $start + $ph.period - 1.0 }
    $t += $ph.period
}
while (((Get-Date) - $t0Wall).TotalSeconds -lt $t) { Start-Sleep -Milliseconds 30 }
$voice.Speak("Done. Hands down. Thank you.")
$meta | ConvertTo-Json -Depth 5 | Out-File "$OutDir\phases.json" -Encoding utf8
$null = $p.WaitForExit(60000)
if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
Get-ChildItem $OutDir | Select-Object Name, Length
