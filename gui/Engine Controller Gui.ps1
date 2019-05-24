Read-Host -Prompt "Press Enter to exit"
$Date = Get-Date
Write-Host "Date: '$Date'"
$DataFileName = Read-Host -Prompt 'Input the data file name'
Write-Host "Launching..."
python series3_hotfire_gui.py '$DataFileName'
Read-Host -Prompt "Press Enter to exit"