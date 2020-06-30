## Credits
[W25Qxx spi flash library](https://github.com/nimaltd/w25qxx) 
I added DMA write and read transfers while still allowing normal SPI transactions. For example, for a DMA write, normal SPI transactions 
are used to check flash busy, enable write, and program the page address. DMA is used for the data buffer write.
