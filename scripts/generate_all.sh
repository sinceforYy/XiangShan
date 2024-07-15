set -e
set -x

# CONFIG=DefaultConfig
CONFIG=MinimalConfig

for module in {NewCSR,}; do
  python3 scripts/parser.py $module --config $CONFIG --prefix "" --xs-home $(pwd)/backend --no-extra-files > generate_$module.log
  mv generate_$module.log bosc_${module}-Release*
done

