# pkg – Package Reference

| File | Description |
|------|-------------|
| `installed_packages.txt` | Snapshot of Python packages installed in the `roboenv` virtual environment |

To recreate the environment:
```bash
python -m venv roboenv
source roboenv/bin/activate   # Linux/Mac
# roboenv\Scripts\activate   # Windows
pip install -r ManualPhase1/WebBasedIKV2/website_dev/requirements.txt
```
