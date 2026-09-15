# -*- coding: utf-8 -*-
import pdfplumber
import sys
sys.stdout.reconfigure(encoding='utf-8')

p = r'C:\Users\HTY\Desktop\历史产品资料4月2日之前\【telesky旗舰店】IBT-2说明书\IBT-2原理图-33.pdf'
with pdfplumber.open(p) as pdf:
    for i, page in enumerate(pdf.pages):
        print(f'--- PAGE {i+1} ---')
        print(page.extract_text() or '(无文字)')
