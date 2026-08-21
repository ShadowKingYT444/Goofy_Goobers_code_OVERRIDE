Keep the normal serial reader in tools/distance_server.py.
Add a fallback path where the Brain program draws a small binary barcode on the Brain screen.
The server captures the Brain screen through COM7, decodes the barcode with Pillow, and feeds those samples into the same graph state.
