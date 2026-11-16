param([ValidateSet("vio", "player", "world")][string]$Part, [string]$Tag = "s2")
# Guided live tests for the hand tracking (spoken prompts, data capture).
# Environment: HAND_TEST_DIR (output folder, default current folder), HAND_MODELS_DIR (folder with the .blob
# models), HAND_TEST_PLAYER (HandTrackingExample.exe, built with ARMLCore.Editor.HandTrackingExample.BuildPlayer).
# Parts: vio = stationary VIO noise runs, player = Unity player hands test with IR off/on,
# world = static hand while the device moves, then a ruler test with the palm on a screen at 30 and 50 cm.
# Analyze with analyze_player.py / analyze_world.py (python analyze_world.py <dir> <tag>).
# Usage: .\guided_tests.ps1 -Part world -Tag s4
$S = $(if ($env:HAND_TEST_DIR) { $env:HAND_TEST_DIR } else { (Get-Location).Path })
$M = $(if ($env:HAND_MODELS_DIR) { $env:HAND_MODELS_DIR } else { "C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker\models" })
$exe = "C:\FuBIntLab\repos\sai-unity\plugin\target\Release\main_depthai.exe"
$player = if ($env:HAND_TEST_PLAYER) { $env:HAND_TEST_PLAYER } else { "$S\unity_test_build\HandTrackingExample.exe" }

Add-Type -AssemblyName System.Speech
$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
function Say([string]$text, [bool]$wait = $true) {
    Write-Host $text
    if ($wait) { $voice.Speak($text) } else { [void]$voice.SpeakAsync($text) }
}
function Clear-SaiEnv {
    foreach ($k in @("SAI_HAND_NO_DEPTH", "SAI_IR_PROJECTOR", "SAI_HAND_REPLAY", "SAI_HAND_LOG", "SAI_USE_STEREO", "SAI_USE_FEATURE_TRACKER",
            "SAI_FAST_VIO", "SAI_HAND_REPLAY_LOOP", "SAI_RECORD_DIR", "SAI_RUN_SECONDS", "SAI_HAND_TRACKING_DEBUG", "SAI_RECORD_NO_COLOR")) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
}

if ($Part -eq "vio") {
    # Stationary camera: VIO noise with/without IR projector, fast VIO, and without hand tracking at all.
    $runs = @(
        @{ n = "control_no_hands"; hands = $false; env = @{} },
        @{ n = "hands_ir0"; hands = $true; env = @{ SAI_IR_PROJECTOR = "0" } },
        @{ n = "hands_ir1"; hands = $true; env = @{ SAI_IR_PROJECTOR = "1" } },
        @{ n = "hands_fastvio"; hands = $true; env = @{ SAI_FAST_VIO = "1" } }
    )
    Say "Please put the device on a stable surface where it cannot move, with nothing in front of the camera. Do not touch it for about four minutes. Starting in ten seconds."
    Start-Sleep -Seconds 10
    foreach ($r in $runs) {
        Clear-SaiEnv
        $dir = "$S\$Tag`_vio_$($r.n)"
        New-Item -ItemType Directory -Force $dir | Out-Null
        Remove-Item "$dir\*" -Force -ErrorAction SilentlyContinue
        $env:SAI_RECORD_DIR = $dir
        $env:SAI_RUN_SECONDS = "50"
        $env:SAI_RECORD_NO_COLOR = "1"
        $env:SAI_HAND_TRACKING_DEBUG = "1"
        if ($r.hands) { $env:SAI_HAND_REPLAY = "$S\rec1\color.raw"; $env:SAI_HAND_REPLAY_LOOP = "1" }
        foreach ($k in $r.env.Keys) { Set-Item "Env:$k" $r.env[$k] }
        $args = if ($r.hands) { @("`"$M\palm_detection_sh4.blob`"", "`"$M\hand_landmark_full_sh4.blob`"") } else { @() }
        Write-Host "== run $($r.n)"
        $p = Start-Process -FilePath $exe -ArgumentList $args -RedirectStandardOutput "$dir\out.txt" -RedirectStandardError "$dir\err.txt" -PassThru -WindowStyle Hidden
        $null = $p.WaitForExit(120000)
        if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
    }
    Clear-SaiEnv
    Say "Finished. You can pick up the device now. Thank you."
}

if ($Part -eq "player") {
    $timeline = @(
        @{ t = 0; say = "Keep your hands out of view." },
        @{ t = 6; say = "Show only your right hand, open palm, and keep it still." },
        @{ t = 14; say = "Now only your left hand." },
        @{ t = 22; say = "Hold your right hand still, forty centimeters from the camera." },
        @{ t = 32; say = "Keep the hand still in the air, or resting on something, and slowly move and rotate the whole device left and right." },
        @{ t = 58; say = "Done. Hands down." }
    )
    $configs = @(
        @{ n = "ir0"; args = @("-handIr", "0", "-handDepth", "0") },
        @{ n = "ir1"; args = @("-handIr", "1", "-handDepth", "1") }
    )
    Say "Next test with the Unity app. Sit as you normally would, holding the device. You will hear instructions. The first run has no IR projector, the second run turns it on."
    foreach ($c in $configs) {
        $log = "$S\$Tag`_player_$($c.n).csv"
        Remove-Item $log -ErrorAction SilentlyContinue
        Say "Starting the $($c.n) run in five seconds."
        Start-Sleep -Seconds 5
        $pargs = @("-screen-fullscreen", "0", "-screen-width", "960", "-screen-height", "540", "-handLog", $log, "-logFile", "$S\$Tag`_player_$($c.n).log") + $c.args
        $p = Start-Process -FilePath $player -ArgumentList $pargs -PassThru
        Start-Sleep -Seconds 9   # window and device start
        $t0 = Get-Date
        foreach ($ph in $timeline) {
            while (((Get-Date) - $t0).TotalSeconds -lt $ph.t) { Start-Sleep -Milliseconds 50 }
            Add-Content "$S\$Tag`_player_$($c.n).phases.txt" ("{0:N1} {1}" -f ((Get-Date) - $p.StartTime).TotalSeconds, $ph.say)
            Say $ph.say $false
        }
        Start-Sleep -Seconds 4
        if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
        Start-Sleep -Seconds 3
    }
    Say "Finished. Thank you."
}

if ($Part -eq "world") {
    $timeline = @(
        @{ t = 0;   say = "Part one. Rest your right hand flat on the table, in view of the camera, and keep it there." },
        @{ t = 9;   say = "Now, holding the device in your other hand, slowly move and rotate it left and right, keeping the hand in view. Keep your right hand still on the table." },
        @{ t = 36;  say = "Part two. Hold the device in your left hand, facing the turned off screen, with the lens exactly thirty centimeters from the screen, measured with the ruler. Press your right palm flat against the screen, in view of the camera. You have forty seconds to set up." },
        @{ t = 78;  say = "Hold as still as you can. Measuring." },
        @{ t = 94;  say = "Part three. Now move the device to exactly fifty centimeters from the screen. Keep your palm flat on the screen. You have thirty seconds." },
        @{ t = 126; say = "Hold as still as you can. Measuring." },
        @{ t = 142; say = "Done. Thank you." }
    )
    $log = "$S\$Tag`_world.csv"
    Remove-Item $log, "$S\$Tag`_world.phases.txt" -ErrorAction SilentlyContinue
    Say "Last test. You will need a ruler or tape measure and a wall. IR projector on. Starting in five seconds."
    Start-Sleep -Seconds 5
    $pargs = @("-screen-fullscreen", "0", "-screen-width", "960", "-screen-height", "540", "-handLog", $log, "-logFile", "$S\$Tag`_world.log", "-handIr", "1", "-handDepth", "1")
    $p = Start-Process -FilePath $player -ArgumentList $pargs -PassThru
    Start-Sleep -Seconds 9
    $t0 = Get-Date
    foreach ($ph in $timeline) {
        while (((Get-Date) - $t0).TotalSeconds -lt $ph.t) { Start-Sleep -Milliseconds 50 }
        Add-Content "$S\$Tag`_world.phases.txt" ("{0:N1} {1}" -f ((Get-Date) - $p.StartTime).TotalSeconds, $ph.say)
        Say $ph.say $false
    }
    Start-Sleep -Seconds 5
    if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
}
