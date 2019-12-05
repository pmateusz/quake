# QUAKE

## Development

```shell
git clone git@github.com:pmateusz/quake.git
mkdir build && cd build
cmake ..
make --jobs 4
```

## Data Sources

Astronomical data about sunrise and sunset were downloaded from [Her Majesty's Nautical Almanac Office](http://astro.ukho.gov.uk/).

## Troubleshooting
1. During build make asks for username and password to a GitHub account

    Ensure SSH Agent is running. The following command should print one process called ssh-agent.
    
    ```shell
   pmateusz@debian:~/dev/quake/build$ ps -p $SSH_AGENT_PID 
     PID TTY          TIME CMD
     788 ?        00:00:00 ssh-agent
   ```

