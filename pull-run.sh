clear
rm pcap/p2p-*
rm positions.txt
git stash
git stash clear
git pull
./ns3 build
./ns3 run main
