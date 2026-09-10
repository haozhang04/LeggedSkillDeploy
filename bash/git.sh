#!/usr/bin/env bash
set -e

Cyan='\033[36m'
Green='\033[32m'
Yellow='\033[33m'
Red='\033[31m'
Reset='\033[0m'

commit_message="$(date '+%Y-%m-%d %H:%M:%S')"

while getopts "m:" opt; do
    case "${opt}" in
        m)
            commit_message="${OPTARG}"
            ;;
    esac
done

printf "%b\n" "${Cyan}==========================================${Reset}"
printf "%b\n" "${Cyan}          Auto Deploy Script${Reset}"
printf "%b\n" "${Cyan}==========================================${Reset}"
printf "\n"

printf "%b\n" "${Yellow}[1/4] Adding changes...${Reset}"
git add .

printf "\n"
printf "%b\n" "${Yellow}[2/4] Committing changes...${Reset}"
git commit -m "${commit_message}"

printf "\n"
printf "%b\n" "${Yellow}[3/4] Pushing to remote...${Reset}"
if ! git push; then
    printf "\n"
    printf "%b\n" "${Red}Error: Git push failed.${Reset}"
    exit 1
fi

printf "\n"
printf "%b\n" "${Green}==========================================${Reset}"
printf "%b\n" "${Green}          Deploy Success!${Reset}"
printf "%b\n" "${Green}==========================================${Reset}"
