# Installation/Configuration and use of MADS

## INI file
 - Default file
 - ini file in the same folder of the broker

## MADS rooms

```bash
mads --rooms
```
## General commands
### MADS echo 

```bash
mads echo --room=Novawalk
```

- Single topic:

```bash
mads echo TOPIC_NAME --room=Novawalk
```

### MADS monitor
```bash
mads sink monitor --room=Novawalk
```

### MADS chat

 - Installation:
```bash
mads package -i mads-chat
```
 - Execution
```bash
mads chat
```

### Rerunner

 - Installation:
```bash
mads package -i rerunner.plugin
```

- [Configuration](https://github.com/MADS-NET/rerunner_plugin)
 - Execution
```bash
mads sink rerunner
```


### MADS record
```bash
mads record -o FILE.bag --room=Novawalk
```
 - info of the record

## Launch
 1. Generate local ini
 2. launch broker
 3. replay the bag:
 ```bash
mads play -i FILE.bag --rate 1
```
 4. Open a monitor
 5. Test previous steps with a director

 ## Create a plugin

 ```bash
mads plugin --type filter --dir average_plugin --install-dir $(mads -p) average
 ```


