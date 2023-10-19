clear
rm pcap/p2p-*
git stash
git stash clear
git pull
./ns3 build
./ns3 run main
