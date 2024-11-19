# ledian: terminal

This project contains the terminal for ledian.
It is a fork of [pangoterm](https://www.leonerd.org.uk/code/pangoterm/), which in turn uses [libvirt](https://www.leonerd.org.uk/code/libvterm/).

## Development shell

```
nix develop ..#pangoterm
make
./pangoterm
```

## Build

```
nix build ..#pangoterm
./result/bin/pangoterm
```
