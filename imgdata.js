const { argv } = require('process');

var fs = require('fs'),
    PNG = require('pngjs').PNG;


if (argv.length !== 4) {
    console.log("Usage: node imgdata.js <image.png> <out.bin>");
    process.exit(1);
}

const imagePath = argv[2];
const outPath = argv[3];

const packedBytes = [];

fs.createReadStream(imagePath)
    .pipe(new PNG())
    .on('parsed', function () {
        // Get the image data
        for (var y = 0; y < this.height; y++) {
            for (var x = 0; x < this.width; x++) {
                var idx = (this.width * y + x) << 2;
                // Get the RGB values
                const B = this.data[idx + 2] / 255;
                const R = this.data[idx] / 255;
                const G = this.data[idx + 1] / 255;

                // Pack the RGB values into a 16-bit value
                const r = R * 0b11111;
                const g = G * 0b11111;
                const b = B * 0b11111;

                // Pack into RGB565 format
                const packed_rgb = (r << 11) | (g << 5) | b;

                // Split the 16-bit value into two 8-bit values
                const packed_byte_0 = packed_rgb & 0xFF;
                const packed_byte_1 = packed_rgb >> 8;

                // Store the packed bytes
                packedBytes.push(packed_byte_1);
                packedBytes.push(packed_byte_0);
            }
        }

        // Write the packed bytes to a file
        const packedBytesBuffer = Buffer.from(packedBytes);
        fs.writeFileSync(outPath, packedBytesBuffer);
    });
