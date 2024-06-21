const { exec } = require('child_process');
const fs = require('fs');
const PNG = require('pngjs').PNG;

const inputPath = 'frames_proc';
const outputPath = 'frames_bin';
const outputRlePath = 'frames_rle';
const outputHeatshrinkPath = 'frames_hs';

class BitArray {
    constructor(bytes = []) {
        this.bytes = bytes;
        this.bitLength = bytes.length << 3;
    }
    pushBit(bit) {
        const byteIndex = this.bitLength >> 3;
        const bitIndex = this.bitLength & 7;
        if (bitIndex === 0) {
            this.bytes.push(0);
        }
        this.bytes[byteIndex] |= bit << (7 - bitIndex);
        this.bitLength++;
    }
    pushBits(bitArray) {
        for (let i = 0; i < bitArray.bitLength; i++) {
            this.pushBit(bitArray.getBit(i));
        }
    }
    setBit(index, bit) {
        const byteIndex = index >> 3;
        const bitIndex = index & 7;
        if (bit) {
            this.bytes[byteIndex] |= 1 << (7 - bitIndex);
        } else {
            this.bytes[byteIndex] &= ~(1 << (7 - bitIndex));
        }
    }
    getBit(index) {
        const byteIndex = index >> 3;
        const bitIndex = index & 7;
        return (this.bytes[byteIndex] >> (7 - bitIndex)) & 1;
    }
    getBytes() {
        return this.bytes;
    }
}

class RunSpan {
    constructor(length, value) {
        this.length = length;
        this.value = value;
    }
    packBits() {
        // Array of boolean values
        const bits = new BitArray();
        // Encode the length as a variable-length quantity
        let length = this.length;
        while (length > 0) {
            // Get the least significant 7 bits
            let byte = length & 0x7F;
            // Shift the length right by 7 bits
            length >>= 7;
            // Set the most significant bit if there are more bytes
            if (length > 0) {
                byte |= 0x80;
            }
            // Store the bits
            for (let i = 0; i < 8; i++) {
                bits.pushBit((byte >> i) & 1 === 1);
            }
        }
        // Encode the value as a 1-bit value
        bits.pushBit(this.value);
        return bits;
    }
}

function runLengthEncode(dataBits) {
    const runs = [];
    let currentRun = new RunSpan(0, dataBits.getBit(0));
    for (let i = 0; i < dataBits.bitLength; i++) {
        const bit = dataBits.getBit(i);
        if (bit === currentRun.value) {
            currentRun.length++;
        } else {
            runs.push(currentRun);
            currentRun = new RunSpan(1, bit);
        }
    }
    runs.push(currentRun);
    return runs;
}

function dataToRleData(dataByteArray) {
    const bits = new BitArray(dataByteArray);
    // console.log(bits);
    const runs = runLengthEncode(bits);
    // console.log(runs);
    const rleBits = new BitArray();
    for (const run of runs) {
        rleBits.pushBits(run.packBits());
    }
    return rleBits.getBytes();
}


// const testdata = [0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b11111111, 0b11111111, 0b11111111, 0b11111100];
// const testresult = dataToRleData(testdata);
// console.log(testresult.map(byte => byte.toString(2).padStart(8, '0')).join(' '));

// process.exit(0);

const apples = {};
const applebytes = {};
let totalapplesize = 0;
const maxapplesize = 7e6;
const appleNames = [];

// let num = 0;

const frameskips = [1]
let frameskipindex = 0;

async function main_async() {
    const frameNames = fs.readdirSync(inputPath);
    const hsFrameNames = fs.readdirSync(outputHeatshrinkPath);

    for (let frameindex = 0; frameindex < frameNames.length;) {
        // num++;

        if (frameindex % 200 == 0) {
            console.log(`Processing frame ${frameindex} of ${frameNames.length}`);
        }

        frameindex += frameskips[frameskipindex];
        frameskipindex = (frameskipindex + 1) % frameskips.length;

        const frameName = frameNames[frameindex];

        const expectedname = `${(frameindex + 1).toString().padStart(5, '0')}.png`;
        if (frameName !== expectedname) {
            console.error(`Expected ${expectedname} but got ${frameName}`);
            continue;
        }

        const frameInputPath = `${inputPath}/${frameName}`;
        const frameOutputPath = `${outputPath}/${frameName.replace('.png', '.bin')}`;
        const frameOutputRlePath = `${outputRlePath}/${frameName.replace('.png', '.rle')}`;
        const frameOutputHeatshrinkPath = `${outputHeatshrinkPath}/${frameName.replace('.png', '.bin')}`;

        if (!hsFrameNames.includes(frameOutputHeatshrinkPath)) {


            const packedBits = [];

            await new Promise((resolve, reject) => {
                fs.createReadStream(frameInputPath).pipe(new PNG()).on('parsed', function () {
                    for (let y = 0; y < this.height; y++) {
                        for (let x = 0; x < this.width; x++) {
                            const idx = (this.width * y + x) << 2;
                            const R = this.data[idx];
                            packedBits.push(+(R > 0));
                        }
                    }
                    resolve();
                }).on('error', function (err) {
                    reject(err);
                });
            });

            const packedBytes = [];
            for (let i = 0; i < packedBits.length; i += 8) {
                let byte = 0;
                for (let j = 0; j < 8; j++) {
                    byte |= (packedBits[i + j] & 1) << (7 - j);
                }
                packedBytes.push(byte);
            }
            const packedBitsBuffer = Buffer.from(packedBytes);
            await fs.promises.writeFile(frameOutputPath, packedBitsBuffer);

            const rleBytes = dataToRleData(packedBytes);
            const rleBytesBuffer = Buffer.from(rleBytes);
            await fs.promises.writeFile(frameOutputRlePath, rleBytesBuffer);

            // if (bundlebytes.length + packedBytes.length < maxbundlesize) {
            //     bundlebytes.push(...packedBytes);
            // } else {
            //     // break;
            // }

            // exec heatshrink --encode
            await new Promise((resolve, reject) => {
                exec(`heatshrink --encode ${frameOutputPath} ${frameOutputHeatshrinkPath}`, (err, stdout, stderr) => {
                    if (err) {
                        reject(err);
                    } else {
                        resolve();
                    }
                });
            });

        }

        // Read the hs file and print out the rust include_bytes!() for it
        const hsBytes = await fs.promises.readFile(frameOutputHeatshrinkPath);
        const hsBytesLength = hsBytes.length;
        const apple_name = `APPLE_FRAME_${frameName.replace('.png', '').toUpperCase()}`;


        // see if there are any previous apples that are the same size
        const sameSizeApples = Object.entries(applebytes).filter(([name, bytes]) => bytes.length === hsBytesLength);
        // see if there are any previous apples that are the same size and the same bytes
        const sameBytesApples = sameSizeApples.filter(([name, bytes]) => Buffer.compare(bytes, hsBytes) === 0);
        if (sameBytesApples.length > 0) {
            const [name, bytes] = sameBytesApples[0];
            appleNames.push(name);
        } else {
            applebytes[apple_name] = hsBytes;
            const apple = `static ${apple_name}: &[u8; ${hsBytesLength}] = include_bytes!("../workspace/${frameOutputHeatshrinkPath}");`;
            apples[apple_name] = apple;
            appleNames.push(apple_name);

            totalapplesize += hsBytesLength;
            if (totalapplesize > maxapplesize) {
                console.log(`Reached max apple size ${maxapplesize} bytes`);
                break;
            }
        }
        // console.log(`Wrote ${frameOutputPath}`);
    }

    // generate the apple rust file
    const individualAppleFrames = Object.values(apples).join('\n');
    const appleNamesArray = appleNames.join(',\n    ');
    const rustfile = `${individualAppleFrames}
pub static APPLE_FRAMES: [&[u8]; ${appleNames.length}] = [
    ${appleNamesArray}
];`;
    await fs.promises.writeFile('../src/apple.rs', rustfile);

    console.log(`Wrote ${appleNames.length} apples with total size ${totalapplesize} bytes`);

    // const bundleBuffer = Buffer.from(bundlebytes);
    // await fs.promises.writeFile('frames_bundle.bin', bundleBuffer);
}

main_async().catch(console.error);
