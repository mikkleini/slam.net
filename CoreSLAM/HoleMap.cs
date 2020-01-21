using System;
using System.Collections.Generic;
using System.Text;

namespace CoreSLAM
{
    /// <summary>
    /// "Attracting hole" map
    /// </summary>
    public class HoleMap
    {
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="sizePixels">Map size in pixels</param>
        /// <param name="sizeMeters">Map size in meters</param>
        public HoleMap(int sizePixels, float sizeMeters)
        {
            Size = sizePixels;
            Scale = sizePixels / sizeMeters;
            Pixels = new ushort[sizePixels * sizePixels];
        }

        /// <summary>
        /// Pixels
        /// </summary>
        public readonly ushort[] Pixels;

        /// <summary>
        /// Map size in pixels
        /// </summary>
        public int Size { get; }

        /// <summary>
        /// Map scale (pixels per meter)
        /// </summary>
        public float Scale { get; }

        /// <summary>
        /// Get packed pixels
        /// Each pixel is packed to 4 bits.
        /// </summary>
        /// <returns></returns>
        public byte[] GetPackedPixels()
        {
            byte[] packed = new byte[Pixels.Length / 2];

            // Get 4-bits per pixel
            for (int i = 0; i < packed.Length; i++)
            {
                packed[i] = (byte)(((Pixels[i * 2] >> 12) << 4) | (Pixels[i * 2 + 1] >> 12));
            }

            return packed;
        }
    }
}
