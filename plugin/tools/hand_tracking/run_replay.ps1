param([string]$Name, [string]$Blob="hand_landmark_full_sh4.blob", [int]$Seconds=30, [string]$Source="slice.raw", [hashtable]$Env=@{})
$S = $(if ($env:HAND_TEST_DIR) { $env:HAND_TEST_DIR } else { (Get-Location).Path })
$M = $(if ($env:HAND_MODELS_DIR) { $env:HAND_MODELS_DIR } else { "C:\Fran\repos\depthai-unity\unity_bridge\depthai_hand_tracker\models" })
$exe="C:\FuBIntLab\repos\sai-unity\plugin\target\Release\main_depthai.exe"
foreach ($k in @("SAI_HAND_NO_DEPTH","SAI_IR_PROJECTOR","SAI_HAND_REPLAY_REALTIME","SAI_RECORD_DIR")) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
$env:SAI_HAND_REPLAY="$S\rec1\$Source"; $env:SAI_HAND_LOG="$S\rec1\log_$Name.jsonl"; $env:SAI_HAND_TRACKING_DEBUG="1"
foreach ($k in $Env.Keys) { Set-Item "Env:$k" $Env[$k] }
$p = Start-Process -FilePath $exe -ArgumentList "`"$M\palm_detection_sh4.blob`"","`"$M\$Blob`"" -RedirectStandardOutput "$S\out_$Name.txt" -RedirectStandardError "$S\err_$Name.txt" -PassThru -WindowStyle Hidden
$null = $p.WaitForExit($Seconds*1000)
if (!$p.HasExited) { Stop-Process -Id $p.Id -Force }
"== $Name"
Get-Content "$S\err_$Name.txt" | Where-Object { $_ -match "fps" } | Select-Object -Skip 1 -First 6
