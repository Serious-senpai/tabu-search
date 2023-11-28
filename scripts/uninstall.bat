@echo off
echo Uninstalling all pip dependencies
for /f %%d in ('pip freeze') do (
    pip uninstall %%d --yes
)
