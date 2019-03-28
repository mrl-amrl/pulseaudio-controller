import time
import subprocess
import rospy


def list_sources():
    process = subprocess.Popen('pactl list sources', shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    out, _ = process.communicate()

    sources = []
    lines = out.split('\n')

    last_source = {}
    for line in lines:
        line = line.strip()

        if line.startswith('Source #'):
            sink_number = line[7:]
            if last_source.get('is_ready', False):
                sources.append(last_source)
                last_source = {}

            last_source["number"] = sink_number

        if line.startswith('Name: '):
            name = line[6:]
            last_source["name"] = name
        if line.startswith('device.description = "'):
            description = line[22:-1]
            last_source["description"] = description
            last_source["is_ready"] = True

    last_source["is_ready"] = True
    sources.append(last_source)
    return sources

def list_sinks():
    process = subprocess.Popen('pactl list sinks', shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    out, _ = process.communicate()

    sinks = []
    lines = out.split('\n')

    last_sink = {}
    for line in lines:
        line = line.strip()

        if line.startswith('Sink #'):
            sink_number = line[6:]
            if last_sink.get('is_ready', False):
                sinks.append(last_sink)
                last_sink = {}

            last_sink["number"] = sink_number

        if line.startswith('Name: '):
            name = line[6:]
            last_sink["name"] = name
        if line.startswith('device.description = "'):
            description = line[13:-1]
            last_sink["description"] = description
            last_sink["is_ready"] = True

    last_sink["is_ready"] = True
    sinks.append(last_sink)
    return sinks


def set_default_sink(name):
    process = subprocess.Popen('pacmd set-default-sink ' + name,
                               shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    _, err = process.communicate()
    if err:
        return False
    return True


def set_default_source(name):
    process = subprocess.Popen('pacmd set-default-source ' + name,
                               shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    _, err = process.communicate()
    if err:
        return False
    return True


def submit_module(module, mode, *args):
    command = [
        "pactl", mode + "-module", module,
    ]
    if args and mode == 'load':
        command.extend(args)

    time.sleep(0.1)

    cmd = " ".join(command)
    rospy.loginfo('Executing {} ...'.format(cmd))

    process = subprocess.Popen(cmd, shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    out, err = process.communicate()
    if err:
        rospy.logerr(err[:-1])
    elif out:
        rospy.loginfo(out[:-1])

    return process.returncode
