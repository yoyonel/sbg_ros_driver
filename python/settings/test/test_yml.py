import yaml, os, re

#define the regex pattern that the parser will use to 'implicitely' tag your node
pattern = re.compile( r'^\<%= ENV\[\'(.*)\'\] %\>(.*)$' )

#now define a custom tag ( say pathex ) and associate the regex pattern we defined
yaml.add_implicit_resolver ( "!pathex", pattern )

#at this point the parser will associate '!pathex' tag whenever the node matches the pattern

#you need to now define a constructor that the parser will invoke
#you can do whatever you want with the node value
def pathex_constructor(loader,node):
  value = loader.construct_scalar(node)
  envVar, remainingPath = pattern.match(value).groups()
  return os.environ[envVar] + remainingPath

#'register' the constructor so that the parser will invoke 'pathex_constructor' for each node '!pathex'
yaml.add_constructor('!pathex', pathex_constructor)

#that is it

data = """
version: 1
disable_existing_loggers: False
formatters:
  precise:
    format: "%(name)-15s # %(levelname)-8s # %(asctime)s # [Line: %(lineno)-3d]: %(message)s"
    datefmt: "%Y-%m-%d %H:%M:%S"
handlers:
  file:
    class:        logging.handlers.RotatingFileHandler
    filename:     <%= ENV['ENV_PROJECT'] %>/target/tracing.log
    encoding:     utf-8
    maxBytes :    10737418244
    backupCount:  7
    formatter:    precise
loggers:
  utility:
    handlers:     [file]
    level:        INFO
    propagate:    True
root:
  handlers:       [file]
  level:          INFO
"""

deSerializedData = yaml.load(data)

print(deSerializedData [ 'handlers'] [ 'file' ] ['filename'] )
print(deSerializedData [ 'root'] ['handlers'])

