// Font structures for newer Adafruit_GFX (1.1 and later).
// Example fonts are included in 'Fonts' directory.
// To use a font in your Arduino sketch, #include the corresponding .h
// file and pass address of GFXfont struct to setFont().  Pass NULL to
// revert to 'classic' fixed-space bitmap font.

/*
font                 xAdvance  yAdvance

FreeMono9pt7b        11        18
FreeMonoBold9pt7b    11        18
FreeMono12pt7b       14        24
FreeMonoBold12pt7b   14        24
FreeMono18pt7b       21        35
FreeMonoBold18pt7b   21        35
FreeMono24pt7b       28        47
FreeMonoBold24pt7b   28        47

FreeSans9pt7b        5..18     22
FreeSansBold9pt7b    5..18     22
FreeSans12pt7b       6..24     29
FreeSans18pt7b       9..36     42
FreeSans24pt7b       12..48    56

FreeSerif9pt7b       5..17     22
*/


#ifndef _GFXFONT_H_
#define _GFXFONT_H_

typedef struct { // Data stored PER GLYPH
	uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
	uint8_t  width, height;    // Bitmap dimensions in pixels
	uint8_t  xAdvance;         // Distance to advance cursor (x axis)
	int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;

typedef struct { // Data stored for FONT AS A WHOLE:
	uint8_t  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	uint8_t   first, last; // ASCII extents
	uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;

#endif // _GFXFONT_H_
