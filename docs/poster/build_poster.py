"""Render poster.html to poster.pdf and poster.png (A1 landscape) with headless Chromium.

    python build_poster.py          (needs: pip install playwright, and a Chromium for Playwright)
"""
import os
from playwright.sync_api import sync_playwright

here = os.path.dirname(os.path.abspath(__file__))
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 3180, "height": 2246})
    pg.goto("file://" + os.path.join(here, "poster.html"))
    pg.wait_for_timeout(500)
    over = pg.evaluate("() => [document.querySelector('.page').scrollHeight, document.querySelector('.page').clientHeight]")
    print("page scroll/client height:", over)
    pg.screenshot(path=os.path.join(here, "poster.png"), full_page=False)
    pg.pdf(path=os.path.join(here, "poster.pdf"), width="841mm", height="594mm", print_background=True)
    b.close()
