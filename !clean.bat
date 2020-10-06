if exist *.uvgui.* del *.uvgui.*
if exist *.bak del *.bak
if exist *.dep del *.dep
for %%a in (.\Objs\*.*) do (if "%%~xa" neq ".???" if "%%~xa" neq ".????" del /Q /F %%a)


